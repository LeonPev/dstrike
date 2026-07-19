from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path

import pytest
import yaml

from src.physics.scenario_config import (
    heading_quaternion_wxyz,
    load_mountain_map_config,
    load_two_drone_config,
    validate_two_drone_config,
)
from src.physics.terrain_map import TerrainMap


ROOT = Path(__file__).resolve().parents[1]
SCENARIO_PATH = ROOT / "config/scenarios/mountain_two_drones.yaml"
MAP_ASSETS = ROOT / "models/scenarios/mountain_map"


@pytest.fixture
def raw() -> dict[str, object]:
    return yaml.safe_load(SCENARIO_PATH.read_text(encoding="utf-8"))


@pytest.fixture
def map_contract() -> tuple[object, TerrainMap]:
    config = load_mountain_map_config(MAP_ASSETS / "scenario.yaml")
    return config, TerrainMap.from_config(config, MAP_ASSETS)


def validate(raw: object, map_contract: tuple[object, TerrainMap]):
    mountain, terrain = map_contract
    return validate_two_drone_config(raw, mountain_config=mountain, terrain=terrain)


def test_canonical_config_resolves_against_completed_map() -> None:
    config = load_two_drone_config(SCENARIO_PATH)
    assert config.target_route_end_xy_m == pytest.approx((130.0, 0.0))
    assert config.route_duration_s == pytest.approx(240.0 / 1.8)
    assert config.route_duration_s >= config.safety.mission_timeout_s + 10.0
    assert config.chaser_drone.initial_agl_m < config.safety.min_terrain_clearance_m
    assert math.dist((0.0, 0.0, 0.0), config.chaser_drone.formation_offset_target_body_m) >= config.safety.min_drone_separation_m


@pytest.mark.parametrize("heading, normalized", [(0.0, 0.0), (90.0, 90.0), (-90.0, 270.0), (180.0, 180.0), (450.0, 90.0)])
def test_heading_normalization_and_conversion(raw, map_contract, heading: float, normalized: float) -> None:
    raw["target_drone"]["heading_deg"] = heading
    # Keep a heading-dependent test route safely inside the map.
    raw["target_drone"]["start_xy_m"] = [0.0, 0.0]
    raw["target_drone"]["route_length_m"] = 140.0
    raw["target_drone"]["speed_mps"] = 1.0
    config = validate(raw, map_contract)
    assert config.target_drone.normalized_heading_deg == normalized
    quaternion = heading_quaternion_wxyz(config.target_drone.heading_deg)
    yaw = math.degrees(2.0 * math.atan2(quaternion[3], quaternion[0])) % 360.0
    assert yaw == pytest.approx(normalized)


@pytest.mark.parametrize(
    "section,key",
    [("target_drone", "speed_mps"), ("target_drone", "route_length_m"), ("safety", "mission_timeout_s"), ("safety", "formation_error_threshold_m"), ("safety", "formation_hold_required_s"), ("visualization", "camera_rate_hz"), ("visualization", "marker_rate_hz")],
)
def test_positive_values_are_required(raw, map_contract, section: str, key: str) -> None:
    raw[section][key] = 0.0
    with pytest.raises(ValueError, match="positive"):
        validate(raw, map_contract)


@pytest.mark.parametrize("section,key", [("target_drone", "heading_deg"), ("chaser_drone", "initial_agl_m"), ("safety", "max_chaser_speed_mps")])
def test_non_finite_values_are_rejected(raw, map_contract, section: str, key: str) -> None:
    raw[section][key] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        validate(raw, map_contract)


def test_unknown_keys_are_rejected_at_every_level(raw, map_contract) -> None:
    for path in (("extra",), ("target_drone", "extra"), ("safety", "extra")):
        changed = deepcopy(raw)
        if len(path) == 1:
            changed[path[0]] = True
        else:
            changed[path[0]][path[1]] = True
        with pytest.raises(ValueError, match="unknown keys"):
            validate(changed, map_contract)


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda value: value.update(name="wrong"), "name must equal"),
        (lambda value: value.update(frame_id="map"), "frame_id must equal"),
        (lambda value: value["map"].update(name="wrong"), "map.name must equal"),
        (lambda value: value["target_drone"].update(route_behavior="loop"), "route_behavior"),
        (lambda value: value["target_drone"].update(altitude_agl_m=-1.0), "nonnegative"),
        (lambda value: value["chaser_drone"].update(initial_agl_m=-1.0), "nonnegative"),
        (lambda value: value["chaser_drone"].update(takeoff_agl_m=0.2), "greater than"),
    ],
)
def test_contract_violations_are_rejected(raw, map_contract, mutate, message: str) -> None:
    mutate(raw)
    with pytest.raises(ValueError, match=message):
        validate(raw, map_contract)


def test_route_duration_buffer_is_required(raw, map_contract) -> None:
    raw["target_drone"]["route_length_m"] = 200.0
    with pytest.raises(ValueError, match="route duration"):
        validate(raw, map_contract)


@pytest.mark.parametrize("point", [[-146.0, 0.0], [0.0, 146.0]])
def test_spawn_points_must_respect_geofence_margin(raw, map_contract, point) -> None:
    raw["chaser_drone"]["start_xy_m"] = point
    with pytest.raises(ValueError, match="geofence margin"):
        validate(raw, map_contract)


def test_full_route_must_respect_geofence_margin(raw, map_contract) -> None:
    raw["target_drone"]["start_xy_m"] = [-100.0, 0.0]
    raw["target_drone"]["route_length_m"] = 250.0
    with pytest.raises(ValueError, match="route end.*geofence margin"):
        validate(raw, map_contract)


def test_target_clearance_and_formation_separation_are_required(raw, map_contract) -> None:
    low_target = deepcopy(raw)
    low_target["target_drone"]["altitude_agl_m"] = 4.9
    with pytest.raises(ValueError, match="minimum terrain clearance"):
        validate(low_target, map_contract)

    close_formation = deepcopy(raw)
    close_formation["chaser_drone"]["formation_offset_target_body_m"] = [1.0, 1.0, 0.0]
    with pytest.raises(ValueError, match="formation offset norm"):
        validate(close_formation, map_contract)
