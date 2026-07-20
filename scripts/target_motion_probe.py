#!/usr/bin/env python3
"""Bounded SCN-004 target-motion verifier using the compiled MuJoCo world."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.scenarios import SCENARIOS_DIR, build_scenario_model_xml, scenario_from_name
from src.physics.target_motion import TargetMotion
from src.physics.terrain_map import TerrainMap


PASS_LINE = "PASS: SCN-004 target drone flies a straight terrain-parallel route"


def angle_error_deg(actual: float, expected: float) -> float:
    return abs((actual - expected + 180.0) % 360.0 - 180.0)


def run(scenario_name: str, duration_s: float) -> None:
    if duration_s <= 0.0 or not math.isfinite(duration_s):
        raise ValueError("duration-s must be positive and finite")
    scenario = scenario_from_name(scenario_name)
    if scenario.config is None or scenario.two_drone_config is None:
        raise ValueError("probe requires a configured two-drone scenario")
    config = scenario.two_drone_config
    terrain = TerrainMap.from_config(scenario.config, SCENARIOS_DIR / "mountain_map")
    quad = MuJoCoQuad(
        model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario),
        vfs_files=scenario.vfs_files,
    )
    motion = TargetMotion(config.target_drone, terrain)
    body_id = int(mujoco.mj_name2id(quad.model, mujoco.mjtObj.mjOBJ_BODY, "target"))
    samples: list[tuple[float, np.ndarray, float]] = []
    sample_period_s = 0.05
    steps_per_sample = max(1, round(sample_period_s / quad.timestep))
    while quad.data.time < duration_s:
        state = motion.apply(quad.model, quad.data, float(quad.data.time))
        samples.append((float(quad.data.time), np.array(quad.data.xpos[body_id]), state.heading_deg))
        for _ in range(steps_per_sample):
            mujoco.mj_step(quad.model, quad.data)
    state = motion.apply(quad.model, quad.data, duration_s)
    samples.append((duration_s, np.array(quad.data.xpos[body_id]), state.heading_deg))

    start_xy = np.asarray(config.target_drone.start_xy_m)
    yaw = math.radians(config.target_drone.normalized_heading_deg)
    along = np.asarray((math.cos(yaw), math.sin(yaw)))
    cross = np.asarray((-along[1], along[0]))
    cross_track = max(abs(float(np.dot(position[:2] - start_xy, cross))) for _, position, _ in samples)
    speeds = [
        float(np.linalg.norm(b[1][:2] - a[1][:2]) / (b[0] - a[0]))
        for a, b in zip(samples, samples[1:]) if b[0] > a[0]
    ]
    speed_error = max(abs(speed - config.target_drone.speed_mps) for speed in speeds)
    agl_errors = [abs(position[2] - terrain.height_at(*position[:2]) - config.target_drone.altitude_agl_m) for _, position, _ in samples]
    heading_error = max(angle_error_deg(heading, config.target_drone.normalized_heading_deg) for _, _, heading in samples)
    minimum_clearance = min(position[2] - terrain.height_at(*position[:2]) for _, position, _ in samples)
    required_duration = config.safety.mission_timeout_s + 10.0

    print(f"samples={len(samples)} duration_s={duration_s:.3f} route_duration_s={config.route_duration_s:.3f}")
    print(f"cross_track_error_m={cross_track:.6f} speed_error_mps={speed_error:.6f}")
    print(f"max_agl_error_m={max(agl_errors):.6f} heading_error_deg={heading_error:.6f} minimum_terrain_clearance_m={minimum_clearance:.6f}")
    assert cross_track <= 0.25
    assert speed_error <= 0.25
    assert max(agl_errors) <= 0.50
    assert heading_error <= 2.0
    assert minimum_clearance >= config.target_drone.altitude_agl_m - 0.50
    assert config.route_duration_s >= required_duration
    print(PASS_LINE)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="mountain_two_drones")
    parser.add_argument("--duration-s", type=float, default=15.0)
    args = parser.parse_args()
    run(args.scenario, args.duration_s)


if __name__ == "__main__":
    main()
