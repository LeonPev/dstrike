#!/usr/bin/env python3
"""Generate all deterministic assets for a configured mountain scenario."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.nature_generator import generate_nature
from src.physics.scenario_config import load_mountain_map_config
from src.physics.terrain_generator import generate_terrain

GENERATED_FILES = (
    "heightfield.bin",
    "heightfield_norm.npy",
    "heightfield_m.npy",
    "terrain_preview.png",
    "nature.json",
    "terrain_meta.json",
)


def generate(scenario: str, output_dir: Path | None = None) -> Path:
    scenario_dir = ROOT / "models/scenarios" / scenario
    config = load_mountain_map_config(scenario_dir / "scenario.yaml")
    if scenario != config.name:
        raise ValueError(f"scenario argument {scenario!r} does not match config name {config.name!r}")
    destination = output_dir or scenario_dir
    destination.mkdir(parents=True, exist_ok=True)

    terrain = generate_terrain(config)
    nature = generate_nature(config, terrain)
    with (destination / "heightfield.bin").open("wb") as stream:
        stream.write(struct.pack("<ii", config.terrain.resolution, config.terrain.resolution))
        stream.write(np.asarray(terrain.height_norm[::-1, :], dtype="<f4").tobytes(order="C"))
    np.save(destination / "heightfield_norm.npy", terrain.height_norm, allow_pickle=False)
    np.save(destination / "heightfield_m.npy", terrain.height_m, allow_pickle=False)

    preview = np.rint(terrain.height_norm.astype(np.float64) * 255.0).astype(np.uint8)
    Image.fromarray(preview).save(destination / "terrain_preview.png", optimize=False, compress_level=9)
    _write_json(destination / "nature.json", nature)

    checksums = {}
    for name in GENERATED_FILES[:-1]:
        checksums[name] = hashlib.sha256((destination / name).read_bytes()).hexdigest()
    metadata = {
        "schema_version": 1,
        "scenario": config.name,
        "generator": "numpy.random.PCG64",
        "terrain_seed": config.terrain.seed,
        "nature_seed": config.nature.seed,
        "resolution": config.terrain.resolution,
        "size_m": config.terrain.size_m,
        "max_height_m": config.terrain.max_height_m,
        "checksums_sha256": checksums,
    }
    _write_json(destination / "terrain_meta.json", metadata)
    return destination


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="mountain_map")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    generate(args.scenario, args.output_dir)


if __name__ == "__main__":
    main()
