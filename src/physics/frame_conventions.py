"""Coordinate conversions between the MuJoCo/ROS and ArduPilot frames."""

from __future__ import annotations

from typing import Any, Iterable

import numpy as np


SQRT_HALF = float(np.sqrt(0.5))

# MuJoCo/ROS world: ENU, z-up. ArduPilot JSON world: NED, z-down.
ENU_TO_NED_QUATERNION_WXYZ = np.array((0.0, SQRT_HALF, SQRT_HALF, 0.0), dtype=float)

# MuJoCo body: FLU, z-up. ArduPilot body: FRD, z-down.
FRD_TO_FLU_QUATERNION_WXYZ = np.array((0.0, 1.0, 0.0, 0.0), dtype=float)


def enu_to_ned_vector(vector: Iterable[float]) -> np.ndarray:
    """Convert an earth-frame vector from east/north/up to north/east/down."""
    east, north, up = _vector3(vector)
    return np.array((north, east, -up), dtype=float)


def flu_to_frd_vector(vector: Iterable[float]) -> np.ndarray:
    """Convert a body-frame vector from forward/left/up to forward/right/down."""
    forward, left, up = _vector3(vector)
    return np.array((forward, -left, -up), dtype=float)


def enu_flu_to_ned_frd_quaternion(quaternion: Iterable[float]) -> np.ndarray:
    """Convert body-to-world orientation from ENU/FLU to NED/FRD.

    MuJoCo stores free-joint quaternions in ``w, x, y, z`` order. ArduPilot's
    JSON parser assigns q1..q4 to its Quaternion type in the same scalar-first
    order, so this function returns ``w, x, y, z``.
    """
    q_enu_flu = _normalise_quaternion_wxyz(quaternion)
    q_ned_frd = _quaternion_multiply_wxyz(
        ENU_TO_NED_QUATERNION_WXYZ,
        _quaternion_multiply_wxyz(q_enu_flu, FRD_TO_FLU_QUATERNION_WXYZ),
    )
    q_ned_frd = _normalise_quaternion_wxyz(q_ned_frd)
    if q_ned_frd[0] < 0.0:
        q_ned_frd = -q_ned_frd
    return q_ned_frd


def ardupilot_state_from_quad_state(state: Any) -> dict[str, object]:
    """Build the mandatory ArduPilot JSON state from a MuJoCo quad state."""
    return {
        "timestamp": float(state.time),
        "imu": {
            "gyro": _float_list(flu_to_frd_vector(state.angular_velocity)),
            "accel_body": _float_list(flu_to_frd_vector(state.body_acceleration)),
        },
        "position": _float_list(enu_to_ned_vector(state.position)),
        "velocity": _float_list(enu_to_ned_vector(state.velocity)),
        "quaternion": _float_list(enu_flu_to_ned_frd_quaternion(state.quaternion)),
    }


def _vector3(vector: Iterable[float]) -> np.ndarray:
    values = np.asarray(tuple(vector), dtype=float)
    if values.shape != (3,):
        raise ValueError("vector must contain exactly three values")
    return values


def _normalise_quaternion_wxyz(quaternion: Iterable[float]) -> np.ndarray:
    values = np.asarray(tuple(quaternion), dtype=float)
    if values.shape != (4,):
        raise ValueError("quaternion must contain exactly four values")
    norm = np.linalg.norm(values)
    if norm <= 0.0:
        raise ValueError("quaternion must have non-zero length")
    return values / norm


def _quaternion_multiply_wxyz(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ),
        dtype=float,
    )


def _float_list(values: Iterable[float]) -> list[float]:
    return [float(value) for value in values]
