from __future__ import annotations

import numpy as np
import pytest

from src.physics.mujoco_quad import MuJoCoQuad


def acceleration_for(thrusts: tuple[float, float, float, float]) -> np.ndarray:
    quad = MuJoCoQuad()
    quad.reset(position=(0.0, 0.0, 2.0))
    quad.step(thrusts)
    return np.array(quad.data.qacc[3:6], dtype=float)


def test_symmetric_command_has_vertical_authority_and_low_angular_acceleration() -> None:
    quad = MuJoCoQuad()
    quad.reset(position=(0.0, 0.0, 2.0))
    hover = quad.mass * abs(float(quad.model.opt.gravity[2])) / 4.0
    quad.step((1.2 * hover,) * 4)
    assert quad.data.qacc[2] > 0.0
    assert np.linalg.norm(quad.data.qacc[3:6]) < 1e-8


@pytest.mark.parametrize(
    ("thrusts", "axis", "sign"),
    (
        ((1.0, 2.0, 2.0, 1.0), 0, 1.0),  # left pair high: +roll
        ((1.0, 2.0, 1.0, 2.0), 1, 1.0),  # rear pair high: +pitch
        ((1.0, 1.0, 2.0, 2.0), 2, 1.0),  # clockwise pair high: +yaw
    ),
)
def test_axis_specific_attitude_response(thrusts, axis, sign) -> None:
    acceleration = acceleration_for(thrusts)
    commanded = acceleration[axis]
    non_commanded = np.delete(acceleration, axis)
    assert np.sign(commanded) == sign
    assert abs(commanded) > 0.05
    assert np.max(np.abs(non_commanded)) < 0.1 * abs(commanded)
