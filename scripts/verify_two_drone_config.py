#!/usr/bin/env python3
"""Print the resolved SCN-001 contract after strict validation."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.scenario_config import load_mountain_map_config, load_two_drone_config
from src.physics.terrain_map import TerrainMap

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    args = parser.parse_args()
    if args.scenario != "mountain_two_drones":
        parser.error("--scenario must be mountain_two_drones")

    scenario_path = ROOT / "config/scenarios" / f"{args.scenario}.yaml"
    map_assets = ROOT / "models/scenarios/mountain_map"
    config = load_two_drone_config(scenario_path)
    mountain = load_mountain_map_config(map_assets / "scenario.yaml")
    terrain = TerrainMap.from_config(mountain, map_assets)

    target_start = config.target_drone.start_xy_m
    target_end = config.target_route_end_xy_m
    chaser_start = config.chaser_drone.start_xy_m
    target_start_z = terrain.height_at(*target_start) + config.target_drone.altitude_agl_m
    target_end_z = terrain.height_at(*target_end) + config.target_drone.altitude_agl_m
    chaser_start_z = terrain.height_at(*chaser_start) + config.chaser_drone.initial_agl_m

    print(f"target spawn xyz_m: ({target_start[0]:.3f}, {target_start[1]:.3f}, {target_start_z:.3f})")
    print(f"chaser spawn xyz_m: ({chaser_start[0]:.3f}, {chaser_start[1]:.3f}, {chaser_start_z:.3f})")
    print(f"target route start xyz_m: ({target_start[0]:.3f}, {target_start[1]:.3f}, {target_start_z:.3f})")
    print(f"target route end xyz_m: ({target_end[0]:.3f}, {target_end[1]:.3f}, {target_end_z:.3f})")
    print(f"route duration_s: {config.route_duration_s:.3f}")
    print(f"mission timeout_s: {config.safety.mission_timeout_s:.3f}")
    print(f"route behavior: {config.target_drone.route_behavior}")
    print(f"target heading_deg normalized: {config.target_drone.normalized_heading_deg:.3f}")
    print(f"chaser heading_deg normalized: {config.chaser_drone.normalized_heading_deg:.3f}")
    print(
        "terrain clearance: chaser initial AGL may be below the minimum; "
        "enforcement begins at the scenario lifecycle gate"
    )
    print("PASS: SCN-001 mountain_two_drones config validates against the completed mountain map")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
