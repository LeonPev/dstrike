from __future__ import annotations

import math

import numpy as np

from src.physics.scenario_config import TargetDroneConfig
from src.physics.ros_visualization import ground_truth_odom_fields
from src.physics.target_motion import TargetMotion
from src.physics.terrain_map import TerrainMap


def motion() -> TargetMotion:
    terrain = TerrainMap(np.zeros((3, 3), dtype=np.float32), 1000.0)
    config = TargetDroneConfig(
        name="target",
        start_xy_m=(10.0, -20.0),
        heading_deg=30.0,
        speed_mps=2.0,
        altitude_agl_m=35.0,
        route_length_m=100.0,
        route_behavior="ping_pong",
    )
    return TargetMotion(config, terrain)


def test_starts_at_configured_pose_and_moves_at_configured_speed() -> None:
    target = motion()
    start = target.state_at(0.0)
    later = target.state_at(10.0)
    assert start.position_xyz == (10.0, -20.0, 35.0)
    assert math.dist(start.position_xyz[:2], later.position_xyz[:2]) == 20.0
    assert math.hypot(*later.velocity_xyz[:2]) == 2.0
    assert later.heading_deg == 30.0


def test_reaches_endpoint_then_reverses_after_overshoot() -> None:
    target = motion()
    endpoint = target.state_at(50.0)
    after = target.state_at(51.0)
    assert math.isclose(endpoint.distance_along_route_m, 100.0, abs_tol=1e-12)
    assert endpoint.direction == -1
    assert endpoint.heading_deg == 210.0
    assert math.isclose(after.distance_along_route_m, 98.0, abs_tol=1e-12)
    assert after.direction == -1
    assert after.heading_deg == 210.0
    assert math.isclose(math.dist(endpoint.position_xyz[:2], after.position_xyz[:2]), 2.0)


def test_crosses_multiple_endpoints_deterministically() -> None:
    target = motion()
    assert target.state_at(100.0).distance_along_route_m == 0.0
    assert target.state_at(100.0).direction == 1
    assert math.isclose(target.state_at(126.0).distance_along_route_m, 52.0)
    assert target.state_at(176.0).direction == -1


def test_follows_terrain_at_constant_agl() -> None:
    heights = np.array([[0.0, 2.0], [4.0, 6.0]], dtype=np.float32)
    terrain = TerrainMap(heights, 100.0)
    config = TargetDroneConfig("target", (-40.0, 0.0), 0.0, 1.0, 12.0, 80.0, "ping_pong")
    target = TargetMotion(config, terrain)
    for time_s in (0.0, 10.0, 40.0, 79.0, 81.0):
        state = target.state_at(time_s)
        assert math.isclose(
            state.position_xyz[2] - terrain.height_at(*state.position_xyz[:2]),
            12.0,
            abs_tol=1e-9,
        )


def test_rejects_invalid_timestamps() -> None:
    target = motion()
    for value in (-1.0, math.inf, math.nan):
        try:
            target.state_at(value)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted invalid timestamp {value}")


def test_motion_state_is_directly_publishable_as_ros_odometry() -> None:
    state = motion().state_at(10.0)

    fields = ground_truth_odom_fields(state)

    assert fields.position_enu == state.position_xyz
    assert fields.linear_velocity_enu == state.velocity_xyz
    assert fields.orientation_xyzw[3] > 0.0
