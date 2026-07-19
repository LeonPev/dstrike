from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from src.physics.mujoco_json import reset_after_frames_from_env
from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.scenario_config import heading_quaternion_wxyz
from src.physics.scenarios import build_scenario_model_xml, scenario_from_name
from src.physics.terrain_map import TerrainMap, physics_height_at


ROOT = Path(__file__).resolve().parents[1]


def test_coordinate_conversion_and_exact_triangle_interpolation() -> None:
    terrain = TerrainMap(np.array([[0.0, 10.0], [20.0, 40.0]], dtype=np.float32), 2.0)
    assert terrain.height_at(-1.0, 1.0) == 0.0
    assert terrain.height_at(1.0, 1.0) == 10.0
    assert terrain.height_at(-1.0, -1.0) == 20.0
    assert terrain.height_at(0.0, 0.5) == 10.0
    assert terrain.height_at(0.5, 0.0) == 20.0


@pytest.mark.parametrize("point", [(-1.01, 0.0), (1.01, 0.0), (0.0, -1.01), (0.0, 1.01)])
def test_out_of_bounds_is_rejected(point: tuple[float, float]) -> None:
    terrain = TerrainMap(np.zeros((2, 2), dtype=np.float32), 2.0)
    with pytest.raises(ValueError, match="outside"):
        terrain.height_at(*point)


@pytest.mark.parametrize("heading", [0.0, 90.0, -90.0, 450.0])
def test_heading_quaternion_conversion(heading: float) -> None:
    quaternion = heading_quaternion_wxyz(heading)
    yaw = math.degrees(2.0 * math.atan2(quaternion[3], quaternion[0])) % 360.0
    assert abs(((yaw - heading + 180.0) % 360.0) - 180.0) <= 1.0


def test_safe_spawn_and_mujoco_terrain_parity_at_sentinel_points() -> None:
    scenario = scenario_from_name("mountain_map")
    assert scenario.config is not None
    terrain = TerrainMap.from_config(scenario.config, scenario.xml_path.parent / scenario.name)
    quad = MuJoCoQuad(
        model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario),
        vfs_files=scenario.vfs_files,
    )
    radius = scenario.config.terrain.size_m / 2.0
    points = [
        (scenario.config.spawn.x_m, scenario.config.spawn.y_m),
        (0.0, 0.0),
        (-radius + 1.0, radius - 1.0),
        (37.25, -61.5),
    ]
    for x_m, y_m in points:
        assert abs(terrain.height_at(x_m, y_m) - physics_height_at(quad, x_m, y_m)) <= 0.10

    spawn = terrain.safe_spawn_pose(scenario.config)
    assert spawn.position_xyz[:2] == (scenario.config.spawn.x_m, scenario.config.spawn.y_m)
    assert abs(
        spawn.position_xyz[2]
        - spawn.terrain_height_m
        - scenario.config.terrain.spawn_clearance_m
    ) < 1e-6
    assert spawn.quaternion_wxyz == scenario.config.spawn.quaternion_wxyz


@pytest.mark.parametrize("raw", ["-1", "1.5", "abc", "+1", "01"])
def test_invalid_reset_interval_is_rejected(raw: str) -> None:
    with pytest.raises(ValueError, match="non-negative integer"):
        reset_after_frames_from_env({"SIM_RESET_AFTER_FRAMES": raw})


def test_reset_interval_zero_is_disabled_and_positive_is_accepted() -> None:
    assert reset_after_frames_from_env({}) == 0
    assert reset_after_frames_from_env({"SIM_RESET_AFTER_FRAMES": "25"}) == 25
