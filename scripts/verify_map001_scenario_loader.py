#!/usr/bin/env python3
"""Bounded acceptance verifier for MAP-001."""

from __future__ import annotations

import copy
from importlib.metadata import version
import math
from pathlib import Path
import sys

import yaml
import mujoco
import numpy
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.scenario_config import (
    heading_quaternion_wxyz,
    validate_mountain_map_config,
)
from src.physics.scenarios import scenario_from_name


CONFIG_PATH = ROOT / "models/scenarios/mountain_map/scenario.yaml"


def expect_invalid(raw: dict, expected_text: str) -> None:
    try:
        validate_mountain_map_config(raw)
    except ValueError as exc:
        assert expected_text in str(exc), (expected_text, str(exc))
    else:
        raise AssertionError(f"expected invalid config containing {expected_text!r}")


def main() -> None:
    assert mujoco.MjVfs is not None
    assert numpy.random.PCG64 is not None
    assert Image.Image is not None
    mountain = scenario_from_name("mountain-map")
    assert mountain.name == "mountain_map"
    assert mountain.xml_path == ROOT / "models/scenarios/mountain_map.xml"
    assert mountain.xml_string == mountain.xml_path.read_text(encoding="utf-8")
    assert mountain.vfs_files == {}
    assert mountain.config is not None
    assert mountain.config.spawn.normalized_heading_deg == 0.0
    assert mountain.config.nature.tree_min_radius_m == 35.0

    for existing_name in (None, "default", "landing_pad", "empty_world", "obstacle_course"):
        existing = scenario_from_name(existing_name)
        assert existing.vfs_files == {}
        assert existing.config is None
        assert existing.xml_string

    raw = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    assert validate_mountain_map_config(raw) == mountain.config

    mutations = (
        (("extra",), 1, "unknown keys"),
        (("schema_version",), 2, "schema_version"),
        (("name",), "other", "name"),
        (("frame_id",), "map", "frame_id"),
        (("replaces_ground_plane",), False, "replaces_ground_plane"),
        (("terrain", "mode"), "download", "terrain.mode"),
        (("terrain", "prng"), "random", "terrain.prng"),
        (("terrain", "quantization_decimals"), 0, "positive"),
        (("terrain", "seed"), 7.0, "integer"),
        (("terrain", "resolution"), 64, "odd"),
        (("terrain", "resolution"), 66, "odd"),
        (("terrain", "rviz_decimation"), 3, "divide"),
        (("terrain", "size_m"), float("nan"), "finite"),
        (("terrain", "max_height_m"), float("inf"), "finite"),
        (("terrain", "base_depth_m"), 0, "positive"),
        (("terrain", "spawn_clearance_m"), -1, "positive"),
        (("terrain", "spawn_basin_radius_m"), 0, "positive"),
        (("terrain", "spawn_basin_max_slope_deg"), -1, "nonnegative"),
        (("terrain", "spawn_basin_max_relief_m"), -1, "nonnegative"),
        (("nature", "seed"), 11.0, "integer"),
        (("nature", "trees"), -1, "nonnegative"),
        (("nature", "rocks"), 1.5, "integer"),
        (("nature", "tree_min_radius_m"), 0, "positive"),
        (("nature", "avoid_spawn_radius_m"), -1, "positive"),
        (("nature", "max_slope_deg"), float("nan"), "finite"),
        (("spawn", "x_m"), 151, "spawn point"),
        (("spawn", "y_m"), -140, "spawn basin"),
        (("spawn", "heading_deg"), float("inf"), "finite"),
    )
    for path, value, message in mutations:
        changed = copy.deepcopy(raw)
        target = changed
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value
        expect_invalid(changed, message)

    sqrt_half = math.sqrt(0.5)
    expected_quaternions = {
        0.0: (1.0, 0.0, 0.0, 0.0),
        90.0: (sqrt_half, 0.0, 0.0, sqrt_half),
        -90.0: (sqrt_half, 0.0, 0.0, -sqrt_half),
    }
    for heading, expected in expected_quaternions.items():
        actual = heading_quaternion_wxyz(heading)
        assert all(math.isclose(a, e, abs_tol=1e-12) for a, e in zip(actual, expected))

    requirements = {}
    for line in (ROOT / "docker/requirements.txt").read_text(encoding="utf-8").splitlines():
        package, pinned = line.split("==", 1)
        requirements[package.lower()] = pinned
    for package in ("mujoco", "numpy", "pillow", "pyyaml"):
        assert package in requirements
        assert version(package) == requirements[package]

    print("PASS: MAP-001 mountain_map is available through the canonical scenario loader")


if __name__ == "__main__":
    main()
