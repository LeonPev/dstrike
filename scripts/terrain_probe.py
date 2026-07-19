#!/usr/bin/env python3
"""Print the configured MAP-004 terrain-aware spawn pose."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.scenarios import scenario_from_name
from src.physics.terrain_map import TerrainMap


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fields", action="store_true")
    args = parser.parse_args()
    scenario = scenario_from_name("mountain_map")
    assert scenario.config is not None
    terrain = TerrainMap.from_config(scenario.config, scenario.xml_path.parent / scenario.name)
    spawn = terrain.safe_spawn_pose(scenario.config)
    if args.fields:
        print(*spawn.position_xyz, spawn.heading_deg)
        return
    print(json.dumps({
        "x_m": spawn.position_xyz[0],
        "y_m": spawn.position_xyz[1],
        "z_m": spawn.position_xyz[2],
        "terrain_height_m": spawn.terrain_height_m,
        "clearance_m": scenario.config.terrain.spawn_clearance_m,
        "heading_deg": spawn.heading_deg,
        "quaternion_wxyz": spawn.quaternion_wxyz,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
