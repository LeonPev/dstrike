from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from src.physics.frame_conventions import (
    ardupilot_state_from_quad_state,
    enu_flu_to_ned_frd_quaternion,
    enu_to_ned_vector,
    flu_to_frd_vector,
)


@dataclass(frozen=True)
class FakeQuadState:
    time: float
    position: np.ndarray
    quaternion: np.ndarray
    velocity: np.ndarray
    angular_velocity: np.ndarray
    body_acceleration: np.ndarray


def test_enu_vectors_convert_to_ned_order() -> None:
    assert np.allclose(enu_to_ned_vector((2.0, 3.0, 4.0)), (3.0, 2.0, -4.0))


def test_flu_body_vectors_convert_to_frd_order() -> None:
    assert np.allclose(flu_to_frd_vector((1.0, 2.0, 3.0)), (1.0, -2.0, -3.0))


def test_ros_z_up_altitude_increase_decreases_ardupilot_down() -> None:
    low_altitude = enu_to_ned_vector((0.0, 0.0, 1.0))[2]
    high_altitude = enu_to_ned_vector((0.0, 0.0, 2.0))[2]

    assert low_altitude == -1.0
    assert high_altitude == -2.0
    assert high_altitude < low_altitude


def test_identity_enu_flu_orientation_is_level_east_heading_in_ned_frd() -> None:
    quaternion = enu_flu_to_ned_frd_quaternion((1.0, 0.0, 0.0, 0.0))

    assert np.allclose(
        quaternion,
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)),
    )
    assert math.isclose(float(np.linalg.norm(quaternion)), 1.0)


def test_ardupilot_state_uses_ned_frd_and_quaternion_only() -> None:
    state = FakeQuadState(
        time=1.25,
        position=np.array((10.0, 20.0, 30.0)),
        quaternion=np.array((1.0, 0.0, 0.0, 0.0)),
        velocity=np.array((1.0, 2.0, 3.0)),
        angular_velocity=np.array((0.1, 0.2, 0.3)),
        body_acceleration=np.array((4.0, 5.0, 6.0)),
    )

    payload = ardupilot_state_from_quad_state(state)

    assert payload["timestamp"] == 1.25
    assert payload["position"] == [20.0, 10.0, -30.0]
    assert payload["velocity"] == [2.0, 1.0, -3.0]
    assert payload["imu"] == {
        "gyro": [0.1, -0.2, -0.3],
        "accel_body": [4.0, -5.0, -6.0],
    }
    assert payload["quaternion"] == [
        math.sqrt(0.5),
        0.0,
        0.0,
        math.sqrt(0.5),
    ]
    assert "attitude" not in payload
