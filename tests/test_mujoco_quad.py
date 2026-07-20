from __future__ import annotations

import numpy as np

from src.physics.frame_conventions import ardupilot_state_from_quad_state
from src.physics.mujoco_quad import MuJoCoQuad


def test_level_reset_reports_gravity_as_body_specific_force() -> None:
    quad = MuJoCoQuad()
    state = quad.reset()
    for _ in range(1000):
        state = quad.step((0.0, 0.0, 0.0, 0.0))

    assert np.allclose(state.body_acceleration, (0.0, 0.0, 9.81), atol=0.1)

    payload = ardupilot_state_from_quad_state(state)
    assert np.allclose(payload["imu"]["accel_body"], (0.0, 0.0, -9.81), atol=0.1)


def test_differential_thrust_changes_attitude() -> None:
    quad = MuJoCoQuad()
    state = quad.reset(position=(0.0, 0.0, 0.2))
    for _ in range(250):
        state = quad.step((0.8, 5.5, 0.9, 5.4))

    assert not np.allclose(state.quaternion, (1.0, 0.0, 0.0, 0.0), atol=0.01)
