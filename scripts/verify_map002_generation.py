#!/usr/bin/env python3
"""Acceptance and determinism checks for MAP-002 generated fixtures."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.generate_terrain_map import GENERATED_FILES, generate
from src.physics.scenario_config import load_mountain_map_config
from src.physics.terrain_generator import bilinear_sample


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="mountain_map")
    args = parser.parse_args()
    committed = ROOT / "models/scenarios" / args.scenario
    config = load_mountain_map_config(committed / "scenario.yaml")

    with tempfile.TemporaryDirectory() as first_raw, tempfile.TemporaryDirectory() as second_raw:
        first, second = Path(first_raw), Path(second_raw)
        generate(args.scenario, first)
        generate(args.scenario, second)
        for name in GENERATED_FILES:
            first_bytes = (first / name).read_bytes()
            assert first_bytes == (second / name).read_bytes(), f"non-deterministic artifact: {name}"
            assert first_bytes == (committed / name).read_bytes(), f"committed fixture differs: {name}"

    untracked = subprocess.run(
        ["git", "ls-files", "--others", "--exclude-standard", "--", str(committed.relative_to(ROOT))],
        cwd=ROOT, check=True, text=True, capture_output=True,
    ).stdout.strip()
    assert not untracked, f"untracked files under scenario directory:\n{untracked}"

    metadata = json.loads((committed / "terrain_meta.json").read_text(encoding="utf-8"))
    assert set(metadata["checksums_sha256"]) == set(GENERATED_FILES[:-1])
    for name, expected in metadata["checksums_sha256"].items():
        assert hashlib.sha256((committed / name).read_bytes()).hexdigest() == expected, name

    norm = np.load(committed / "heightfield_norm.npy", allow_pickle=False)
    height_m = np.load(committed / "heightfield_m.npy", allow_pickle=False)
    expected_shape = (config.terrain.resolution, config.terrain.resolution)
    assert norm.shape == height_m.shape == expected_shape
    assert norm.dtype == height_m.dtype == np.dtype("<f4")
    assert np.isfinite(norm).all() and np.isfinite(height_m).all()
    assert float(norm.min()) == 0.0 and float(norm.max()) == 1.0
    assert float(height_m.min()) >= 0.0 and float(height_m.max()) <= config.terrain.max_height_m
    assert np.array_equal(
        height_m,
        np.asarray(np.round(norm.astype(np.float64) * config.terrain.max_height_m, config.terrain.quantization_decimals), dtype="<f4"),
    )
    assert len(set(float(value) for value in (norm[0, 0], norm[0, -1], norm[-1, 0], norm[-1, -1]))) == 4

    binary = (committed / "heightfield.bin").read_bytes()
    nrow, ncol = struct.unpack("<ii", binary[:8])
    assert (nrow, ncol) == expected_shape
    payload = np.frombuffer(binary[8:], dtype="<f4").reshape(expected_shape)
    assert np.array_equal(payload, norm[::-1, :])
    assert not np.array_equal(payload, norm)

    spacing = config.terrain.size_m / (config.terrain.resolution - 1)
    axis = np.linspace(-config.terrain.size_m / 2.0, config.terrain.size_m / 2.0, config.terrain.resolution)
    x_grid, y_grid = np.meshgrid(axis, axis)
    basin = np.hypot(x_grid - config.spawn.x_m, y_grid - config.spawn.y_m) <= config.terrain.spawn_basin_radius_m
    assert basin.any()
    basin_values = height_m[basin].astype(np.float64)
    assert np.ptp(basin_values) <= config.terrain.spawn_basin_max_relief_m
    dy, dx = np.gradient(height_m.astype(np.float64), spacing, spacing)
    slopes = np.degrees(np.arctan(np.hypot(dx, dy)))
    assert float(slopes[basin].max()) <= config.terrain.spawn_basin_max_slope_deg
    assert math.hypot(config.spawn.x_m - config.spawn.x_m, config.spawn.y_m - config.spawn.y_m) <= config.terrain.spawn_basin_radius_m

    nature = json.loads((committed / "nature.json").read_text(encoding="utf-8"))
    assert len(nature["trees"]) == config.nature.trees
    assert len(nature["rocks"]) == config.nature.rocks
    ids: set[str] = set()
    half = config.terrain.size_m / 2.0
    for kind, objects in (("tree", nature["trees"]), ("rock", nature["rocks"])):
        for index, item in enumerate(objects):
            assert item["id"] == f"{kind}-{index:03d}" and item["id"] not in ids
            ids.add(item["id"])
            assert -half <= item["x_m"] <= half and -half <= item["y_m"] <= half
            assert math.hypot(item["x_m"] - config.spawn.x_m, item["y_m"] - config.spawn.y_m) >= config.nature.avoid_spawn_radius_m
            if kind == "tree":
                assert math.hypot(item["x_m"], item["y_m"]) >= config.nature.tree_min_radius_m
            assert bilinear_sample(slopes, item["x_m"], item["y_m"], config.terrain.size_m) <= config.nature.max_slope_deg
            expected_z = bilinear_sample(height_m, item["x_m"], item["y_m"], config.terrain.size_m)
            assert math.isclose(item["z_m"], expected_z, abs_tol=1.1e-6)
            assert 0.0 <= item["yaw_deg"] <= 360.0 and item["scale"] > 0.0

    print("PASS: MAP-002 deterministic mountain terrain and nature assets generated")


if __name__ == "__main__":
    main()
