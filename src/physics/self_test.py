"""Standalone MuJoCo quadrotor self-test for DFD-006."""

from __future__ import annotations

import argparse

import numpy as np

from src.physics.mujoco_quad import MuJoCoQuad


GRAVITY_MPS2 = 9.80665


def run_case(quad: MuJoCoQuad, thrusts: tuple[float, float, float, float], duration: float):
    start = quad.reset()
    steps = max(1, round(duration / quad.timestep))
    samples = [start.position[2]]

    state = start
    for _ in range(steps):
        state = quad.step(thrusts)
        samples.append(state.position[2])

    return start, state, np.asarray(samples)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the DFD-006 MuJoCo self-test.")
    parser.add_argument("--duration", type=float, default=0.5)
    args = parser.parse_args(argv)

    quad = MuJoCoQuad()

    zero_start, zero_end, _ = run_case(quad, (0.0, 0.0, 0.0, 0.0), args.duration)
    zero_delta = zero_end.position[2] - zero_start.position[2]
    if not zero_end.position[2] < zero_start.position[2] - 0.05:
        raise RuntimeError(
            "zero thrust case failed: expected altitude to decrease, "
            f"start_z={zero_start.position[2]:.4f} end_z={zero_end.position[2]:.4f}"
        )

    hover_thrust = quad.mass * GRAVITY_MPS2 / 4.0
    hover_start, hover_end, hover_samples = run_case(
        quad,
        (hover_thrust, hover_thrust, hover_thrust, hover_thrust),
        args.duration,
    )
    hover_delta = hover_end.position[2] - hover_start.position[2]
    max_hover_drift = float(np.max(np.abs(hover_samples - hover_start.position[2])))
    if hover_end.position[2] < hover_start.position[2] - 0.05 or max_hover_drift > 0.10:
        raise RuntimeError(
            "hover-ish thrust case failed: expected altitude to remain bounded, "
            f"start_z={hover_start.position[2]:.4f} "
            f"end_z={hover_end.position[2]:.4f} "
            f"max_drift={max_hover_drift:.4f}"
        )

    print(
        "zero thrust: altitude decreases "
        f"start_z={zero_start.position[2]:.4f} "
        f"end_z={zero_end.position[2]:.4f} "
        f"delta_z={zero_delta:.4f}"
    )
    print(
        "hover-ish thrust: altitude remains bounded "
        f"per_motor_thrust_n={hover_thrust:.4f} "
        f"start_z={hover_start.position[2]:.4f} "
        f"end_z={hover_end.position[2]:.4f} "
        f"delta_z={hover_delta:.4f} "
        f"max_drift_m={max_hover_drift:.4f}"
    )
    print("PASS: DFD-006 MuJoCo quad self-test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
