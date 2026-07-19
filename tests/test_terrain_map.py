from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from src.physics.mujoco_json import reset_after_frames_from_env, scenario_spawn_pose
from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.scenario_config import heading_quaternion_wxyz
from src.physics.scenarios import (
    build_scenario_model_xml,
    mountain_launch_pad_top_z,
    scenario_from_name,
)
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

    spawn = scenario_spawn_pose(scenario)
    launch_pad_top_z = mountain_launch_pad_top_z(scenario)
    assert spawn.position_xyz[:2] == (scenario.config.spawn.x_m, scenario.config.spawn.y_m)
    assert abs(
        spawn.position_xyz[2]
        - launch_pad_top_z
        - scenario.config.terrain.spawn_clearance_m
    ) < 1e-6
    assert spawn.launch_surface_height_m == launch_pad_top_z
    assert launch_pad_top_z > spawn.terrain_height_m
    assert spawn.quaternion_wxyz == scenario.config.spawn.quaternion_wxyz


def test_mountain_launch_pad_provides_stable_level_contact() -> None:
    scenario = scenario_from_name("mountain_map")
    model_xml = build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario)
    quad = MuJoCoQuad(model_xml=model_xml, vfs_files=scenario.vfs_files)
    spawn = scenario_spawn_pose(scenario)
    state = quad.reset(spawn.position_xyz, spawn.quaternion_wxyz)

    for _ in range(1000):
        state = quad.step((0.0, 0.0, 0.0, 0.0))

    pad_id = int(
        mujoco.mj_name2id(
            quad.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "scenario_landing_pad",
        )
    )
    terrain_id = int(
        mujoco.mj_name2id(
            quad.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "mountain_terrain",
        )
    )
    drone_geom_ids = {
        geom_id
        for geom_id, body_id in enumerate(quad.model.geom_bodyid)
        if int(body_id) == quad.body_id
    }
    contact_pairs = {
        frozenset(
            (
                int(quad.data.contact[index].geom1),
                int(quad.data.contact[index].geom2),
            )
        )
        for index in range(quad.data.ncon)
    }
    assert any(
        pad_id in pair and not pair.isdisjoint(drone_geom_ids)
        for pair in contact_pairs
    )
    assert not any(
        terrain_id in pair and not pair.isdisjoint(drone_geom_ids)
        for pair in contact_pairs
    )

    roll_deg, pitch_deg = _roll_pitch_degrees(state.quaternion)
    assert abs(roll_deg) <= 0.5
    assert abs(pitch_deg) <= 0.5
    assert np.linalg.norm(state.position[:2] - np.asarray(spawn.position_xyz[:2])) <= 0.02
    assert np.linalg.norm(state.velocity) <= 0.02
    assert np.linalg.norm(state.angular_velocity) <= 0.05

    body_geom_id = int(
        mujoco.mj_name2id(quad.model, mujoco.mjtObj.mjOBJ_GEOM, "body")
    )
    body_bottom_offset_m = (
        float(quad.model.geom_pos[body_geom_id, 2])
        - float(quad.model.geom_size[body_geom_id, 2])
    )
    expected_rest_z = mountain_launch_pad_top_z(scenario) - body_bottom_offset_m
    assert abs(float(state.position[2]) - expected_rest_z) <= 0.005


def test_level_mountain_pad_preserves_vertical_up_and_down_motion() -> None:
    scenario = scenario_from_name("mountain_map")
    quad = MuJoCoQuad(
        model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario),
        vfs_files=scenario.vfs_files,
    )
    spawn = scenario_spawn_pose(scenario)
    state = quad.reset(spawn.position_xyz, spawn.quaternion_wxyz)
    for _ in range(1000):
        state = quad.step((0.0, 0.0, 0.0, 0.0))

    rest_position = np.array(state.position, copy=True)
    hover_thrust = quad.mass * abs(float(quad.model.opt.gravity[2])) / 4.0
    states = []
    for _ in range(400):
        state = quad.step((1.25 * hover_thrust,) * 4)
        states.append(state)
    up_end_z = float(state.position[2])
    for _ in range(200):
        state = quad.step((0.0, 0.0, 0.0, 0.0))
        states.append(state)

    peak_z = max(float(sample.position[2]) for sample in states)
    down_end_z = float(state.position[2])
    assert up_end_z - float(rest_position[2]) >= 0.70
    assert peak_z - down_end_z >= 0.15
    assert float(state.velocity[2]) <= -1.0
    assert down_end_z >= mountain_launch_pad_top_z(scenario) + 0.50
    assert max(
        np.linalg.norm(sample.position[:2] - rest_position[:2])
        for sample in states
    ) <= 0.05
    assert max(
        max(abs(value) for value in _roll_pitch_degrees(sample.quaternion))
        for sample in states
    ) <= 1.0
    assert all(
        np.isfinite(
            np.concatenate(
                (
                    sample.position,
                    sample.quaternion,
                    sample.velocity,
                    sample.angular_velocity,
                )
            )
        ).all()
        for sample in states
    )


@pytest.mark.parametrize("raw", ["-1", "1.5", "abc", "+1", "01"])
def test_invalid_reset_interval_is_rejected(raw: str) -> None:
    with pytest.raises(ValueError, match="non-negative integer"):
        reset_after_frames_from_env({"SIM_RESET_AFTER_FRAMES": raw})


def test_reset_interval_zero_is_disabled_and_positive_is_accepted() -> None:
    assert reset_after_frames_from_env({}) == 0
    assert reset_after_frames_from_env({"SIM_RESET_AFTER_FRAMES": "25"}) == 25


def _roll_pitch_degrees(quaternion_wxyz: np.ndarray) -> tuple[float, float]:
    w, x, y, z = (float(value) for value in quaternion_wxyz)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sine = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(roll), math.degrees(math.asin(pitch_sine))
