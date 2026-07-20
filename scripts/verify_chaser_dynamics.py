#!/usr/bin/env python3
"""Bounded SCN-003 chaser force/torque authority verifier."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.mujoco_quad import MuJoCoQuad


CASES = {
    "roll": ((1.0, 2.0, 2.0, 1.0), 0),
    "pitch": ((1.0, 2.0, 1.0, 2.0), 1),
    "yaw": ((1.0, 1.0, 2.0, 2.0), 2),
}


def main() -> None:
    quad = MuJoCoQuad()
    quad.reset(position=(0.0, 0.0, 2.0))
    hover = quad.mass * abs(float(quad.model.opt.gravity[2])) / 4.0
    quad.step((1.2 * hover,) * 4)
    vertical = float(quad.data.qacc[2])
    angular = np.array(quad.data.qacc[3:6], dtype=float)
    assert vertical > 0.0
    assert np.linalg.norm(angular) < 1e-8
    print(f"symmetric vertical_accel={vertical:.6f} angular_norm={np.linalg.norm(angular):.6f}")

    for name, (thrusts, axis) in CASES.items():
        quad.reset(position=(0.0, 0.0, 2.0))
        quad.step(thrusts)
        response = np.array(quad.data.qacc[3:6], dtype=float)
        commanded = abs(float(response[axis]))
        cross_axis = float(np.max(np.abs(np.delete(response, axis))))
        assert response[axis] > 0.05
        assert cross_axis < 0.1 * commanded
        print(
            f"{name} angular_accel={response.tolist()} "
            f"cross_axis_ratio={cross_axis / commanded:.6f}"
        )

    print("PASS: SCN-003 chaser dynamics support lateral, yaw, and vertical control")


if __name__ == "__main__":
    main()
