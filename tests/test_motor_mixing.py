from __future__ import annotations

import numpy as np
import pytest

from src.physics.motor_model import body_wrench


POSITIONS = (
    (0.18, -0.18, 0.02),  # motor 1: front right
    (-0.18, 0.18, 0.02),  # motor 2: rear left
    (0.18, 0.18, 0.02),   # motor 3: front left
    (-0.18, -0.18, 0.02), # motor 4: rear right
)


@pytest.mark.parametrize(
    ("thrusts", "axis", "sign"),
    (
        ((1.0, 2.0, 2.0, 1.0), 0, 1.0),
        ((1.0, 2.0, 1.0, 2.0), 1, 1.0),
        ((1.0, 1.0, 2.0, 2.0), 2, 1.0),
    ),
)
def test_axis_mix_has_expected_sign(thrusts, axis, sign) -> None:
    _, torque = body_wrench(thrusts, POSITIONS)
    assert np.sign(torque[axis]) == sign


def test_symmetric_mix_has_only_vertical_force() -> None:
    force, torque = body_wrench((2.0,) * 4, POSITIONS)
    assert np.allclose(force, (0.0, 0.0, 8.0))
    assert np.allclose(torque, (0.0, 0.0, 0.0))


def test_invalid_mix_is_rejected() -> None:
    with pytest.raises(ValueError):
        body_wrench((1.0, -1.0, 1.0, 1.0), POSITIONS)
