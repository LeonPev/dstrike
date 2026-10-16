"""Quad-X rotor layout and force/torque mixing.

ArduPilot's first four JSON PWM outputs use Quad-X motor order:

1. front right (counter-clockwise)
2. rear left (counter-clockwise)
3. front left (clockwise)
4. rear right (clockwise)

Positions use the MuJoCo body convention (+X forward, +Y left, +Z up).
``yaw_torque_sign`` is the reaction torque on the body around +Z.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


DEFAULT_YAW_MOMENT_ARM_M = 0.02


@dataclass(frozen=True)
class Rotor:
    channel: int
    site_suffix: str
    yaw_torque_sign: float


ARDUPILOT_QUAD_X_ROTORS = (
    Rotor(1, "rotor_front_right", -1.0),
    Rotor(2, "rotor_rear_left", -1.0),
    Rotor(3, "rotor_front_left", 1.0),
    Rotor(4, "rotor_rear_right", 1.0),
)


def body_wrench(
    thrusts: Iterable[float],
    rotor_positions: Iterable[Iterable[float]],
    yaw_moment_arm_m: float = DEFAULT_YAW_MOMENT_ARM_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Return total body-frame force and torque for the ordered rotors."""
    thrust = np.asarray(tuple(thrusts), dtype=float)
    positions = np.asarray(tuple(tuple(position) for position in rotor_positions), dtype=float)
    if thrust.shape != (4,):
        raise ValueError("thrusts must contain exactly four values")
    if positions.shape != (4, 3):
        raise ValueError("rotor_positions must contain four xyz vectors")
    if np.any(thrust < 0.0) or not np.all(np.isfinite(thrust)):
        raise ValueError("thrusts must be finite and non-negative")
    if yaw_moment_arm_m < 0.0 or not np.isfinite(yaw_moment_arm_m):
        raise ValueError("yaw_moment_arm_m must be finite and non-negative")

    forces = np.zeros((4, 3), dtype=float)
    forces[:, 2] = thrust
    torque = np.sum(np.cross(positions, forces), axis=0)
    torque[2] += yaw_moment_arm_m * sum(
        rotor.yaw_torque_sign * value
        for rotor, value in zip(ARDUPILOT_QUAD_X_ROTORS, thrust)
    )
    return np.sum(forces, axis=0), torque
