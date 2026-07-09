from __future__ import annotations

import math
from pathlib import Path
import sys


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src" / "ros2" / "drone_bridge"),
)

from drone_bridge.frame_conventions import (  # noqa: E402
    GlobalPositionInt,
    GlobalPositionOrigin,
    LocalPositionNed,
    attitude_rpy_to_quaternion_wxyz,
    global_position_int_to_local_ned,
    local_position_ned_to_ros_odom_fields,
    ned_frd_to_enu_flu_quaternion_xyzw,
    ned_to_enu_vector,
    ros_enu_twist_to_mavlink_ned_velocity,
    ros_yaw_rate_to_mavlink_ned,
)


def test_ned_position_converts_to_ros_enu_z_up() -> None:
    assert ned_to_enu_vector((10.0, 20.0, -3.0)) == (20.0, 10.0, 3.0)


def test_local_position_ned_converts_to_ros_odometry_fields() -> None:
    fields = local_position_ned_to_ros_odom_fields(
        LocalPositionNed(
            x=10.0,
            y=20.0,
            z=-3.0,
            vx=1.0,
            vy=2.0,
            vz=-0.5,
        )
    )

    assert fields.position_enu == (20.0, 10.0, 3.0)
    assert fields.linear_velocity_enu == (2.0, 1.0, 0.5)
    assert fields.orientation_xyzw == (
        0.0,
        0.0,
        math.sqrt(0.5),
        math.sqrt(0.5),
    )


def test_mavlink_north_heading_becomes_ros_positive_yaw() -> None:
    quaternion_xyzw = ned_frd_to_enu_flu_quaternion_xyzw((1.0, 0.0, 0.0, 0.0))

    assert quaternion_xyzw == (
        0.0,
        0.0,
        math.sqrt(0.5),
        math.sqrt(0.5),
    )


def test_mavlink_east_heading_becomes_ros_identity_heading() -> None:
    attitude = attitude_rpy_to_quaternion_wxyz(0.0, 0.0, math.pi * 0.5)
    quaternion_xyzw = ned_frd_to_enu_flu_quaternion_xyzw(attitude)

    assert all(
        math.isclose(actual, expected, abs_tol=1e-12)
        for actual, expected in zip(quaternion_xyzw, (0.0, 0.0, 0.0, 1.0))
    )


def test_ros_cmd_vel_converts_to_mavlink_guided_velocity() -> None:
    assert ros_enu_twist_to_mavlink_ned_velocity((4.0, 5.0, 6.0)) == (
        5.0,
        4.0,
        -6.0,
    )
    assert ros_yaw_rate_to_mavlink_ned(0.25) == -0.25


def test_global_position_int_fallback_anchors_local_ned_at_first_fix() -> None:
    origin = GlobalPositionOrigin(lat_deg=-35.0, lon_deg=149.0)
    local_position, same_origin = global_position_int_to_local_ned(
        GlobalPositionInt(
            lat_e7=int((-35.0 + math.degrees(10.0 / 6_378_137.0)) * 10_000_000),
            lon_e7=1490000000,
            relative_alt_mm=2500,
            vx_cm_s=100,
            vy_cm_s=200,
            vz_cm_s=-50,
        ),
        origin,
    )

    assert same_origin == origin
    assert math.isclose(local_position.x, 10.0, abs_tol=0.02)
    assert math.isclose(local_position.y, 0.0, abs_tol=0.02)
    assert local_position.z == -2.5
    assert local_position.vx == 1.0
    assert local_position.vy == 2.0
    assert local_position.vz == -0.5
