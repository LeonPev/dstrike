"""Deterministic tree and rock placement for generated terrain."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from src.physics.scenario_config import MountainMapConfig
from src.physics.terrain_generator import TerrainAssets, bilinear_sample


def generate_nature(config: MountainMapConfig, terrain: TerrainAssets) -> dict[str, Any]:
    rng = np.random.Generator(np.random.PCG64(config.nature.seed))
    result: dict[str, Any] = {"schema_version": 1, "trees": [], "rocks": []}
    result["trees"] = _place("tree", config.nature.trees, config, terrain, rng)
    result["rocks"] = _place("rock", config.nature.rocks, config, terrain, rng)
    return result


def _place(
    kind: str,
    count: int,
    config: MountainMapConfig,
    terrain: TerrainAssets,
    rng: np.random.Generator,
) -> list[dict[str, float | str]]:
    placed: list[dict[str, float | str]] = []
    half = config.terrain.size_m / 2.0
    margin = 2.0
    for index in range(count):
        for _ in range(20_000):
            x_m = float(rng.uniform(-half + margin, half - margin))
            y_m = float(rng.uniform(-half + margin, half - margin))
            if math.hypot(x_m - config.spawn.x_m, y_m - config.spawn.y_m) < config.nature.avoid_spawn_radius_m:
                continue
            if kind == "tree" and math.hypot(x_m, y_m) < config.nature.tree_min_radius_m:
                continue
            slope = bilinear_sample(terrain.slope_deg, x_m, y_m, config.terrain.size_m)
            if slope > config.nature.max_slope_deg:
                continue
            z_m = bilinear_sample(terrain.height_m, x_m, y_m, config.terrain.size_m)
            placed.append(
                {
                    "id": f"{kind}-{index:03d}",
                    "x_m": round(x_m, 6),
                    "y_m": round(y_m, 6),
                    "z_m": round(z_m, 6),
                    "yaw_deg": round(float(rng.uniform(0.0, 360.0)), 6),
                    "scale": round(float(rng.uniform(0.8, 1.25) if kind == "tree" else rng.uniform(0.5, 1.8)), 6),
                }
            )
            break
        else:
            raise RuntimeError(f"could not place {kind} {index} within configured constraints")
    return placed
