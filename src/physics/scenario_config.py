"""Strict configuration contract for file-backed map scenarios."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

import yaml


@dataclass(frozen=True)
class TerrainConfig:
    mode: str
    prng: str
    quantization_decimals: int
    seed: int
    size_m: float
    resolution: int
    max_height_m: float
    base_depth_m: float
    spawn_clearance_m: float
    rviz_decimation: int
    spawn_basin_radius_m: float
    spawn_basin_max_slope_deg: float
    spawn_basin_max_relief_m: float


@dataclass(frozen=True)
class NatureConfig:
    seed: int
    trees: int
    rocks: int
    # Every generated tree trunk center must have hypot(x, y) >= this radius.
    tree_min_radius_m: float
    avoid_spawn_radius_m: float
    max_slope_deg: float


@dataclass(frozen=True)
class SpawnConfig:
    x_m: float
    y_m: float
    heading_deg: float

    @property
    def normalized_heading_deg(self) -> float:
        return self.heading_deg % 360.0

    @property
    def quaternion_wxyz(self) -> tuple[float, float, float, float]:
        return heading_quaternion_wxyz(self.heading_deg)


@dataclass(frozen=True)
class MountainMapConfig:
    schema_version: int
    name: str
    frame_id: str
    replaces_ground_plane: bool
    terrain: TerrainConfig
    nature: NatureConfig
    spawn: SpawnConfig


def heading_quaternion_wxyz(heading_deg: float) -> tuple[float, float, float, float]:
    """Convert z-up yaw (0 = +X, positive CCW) to a wxyz quaternion."""
    heading = _finite_number(heading_deg, "heading_deg") % 360.0
    # Use the equivalent signed yaw to keep the quaternion's scalar component
    # nonnegative and make -90 degrees produce the intuitive negative-z form.
    signed_heading = heading - 360.0 if heading > 180.0 else heading
    half_yaw = math.radians(signed_heading) / 2.0
    return (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))


def load_mountain_map_config(path: str | Path) -> MountainMapConfig:
    with Path(path).open("r", encoding="utf-8") as config_file:
        raw = yaml.safe_load(config_file)
    return validate_mountain_map_config(raw)


def validate_mountain_map_config(raw: object) -> MountainMapConfig:
    root = _mapping(raw, "scenario")
    _keys(root, {"schema_version", "name", "frame_id", "replaces_ground_plane", "terrain", "nature", "spawn"}, "scenario")

    schema_version = _integer(root.get("schema_version"), "schema_version")
    if schema_version != 1:
        raise ValueError("schema_version must equal 1")
    if root.get("name") != "mountain_map":
        raise ValueError("name must equal mountain_map")
    if root.get("frame_id") != "world":
        raise ValueError("frame_id must equal world")
    if root.get("replaces_ground_plane") is not True:
        raise ValueError("replaces_ground_plane must be true")

    terrain_raw = _mapping(root.get("terrain"), "terrain")
    terrain_keys = {"mode", "prng", "quantization_decimals", "seed", "size_m", "resolution", "max_height_m", "base_depth_m", "spawn_clearance_m", "rviz_decimation", "spawn_basin_radius_m", "spawn_basin_max_slope_deg", "spawn_basin_max_relief_m"}
    _keys(terrain_raw, terrain_keys, "terrain")
    if terrain_raw.get("mode") != "procedural":
        raise ValueError("terrain.mode must equal procedural")
    if terrain_raw.get("prng") != "numpy.random.PCG64":
        raise ValueError("terrain.prng must equal numpy.random.PCG64")

    quantization_decimals = _positive_integer(terrain_raw.get("quantization_decimals"), "terrain.quantization_decimals")
    seed = _integer(terrain_raw.get("seed"), "terrain.seed")
    resolution = _integer(terrain_raw.get("resolution"), "terrain.resolution")
    if resolution < 65 or resolution % 2 == 0:
        raise ValueError("terrain.resolution must be odd and at least 65")
    rviz_decimation = _positive_integer(terrain_raw.get("rviz_decimation"), "terrain.rviz_decimation")
    if (resolution - 1) % rviz_decimation:
        raise ValueError("terrain.rviz_decimation must divide resolution - 1")

    terrain = TerrainConfig(
        mode="procedural", prng="numpy.random.PCG64",
        quantization_decimals=quantization_decimals, seed=seed,
        size_m=_positive(terrain_raw.get("size_m"), "terrain.size_m"),
        resolution=resolution,
        max_height_m=_positive(terrain_raw.get("max_height_m"), "terrain.max_height_m"),
        base_depth_m=_positive(terrain_raw.get("base_depth_m"), "terrain.base_depth_m"),
        spawn_clearance_m=_positive(terrain_raw.get("spawn_clearance_m"), "terrain.spawn_clearance_m"),
        rviz_decimation=rviz_decimation,
        spawn_basin_radius_m=_positive(terrain_raw.get("spawn_basin_radius_m"), "terrain.spawn_basin_radius_m"),
        spawn_basin_max_slope_deg=_nonnegative(terrain_raw.get("spawn_basin_max_slope_deg"), "terrain.spawn_basin_max_slope_deg"),
        spawn_basin_max_relief_m=_nonnegative(terrain_raw.get("spawn_basin_max_relief_m"), "terrain.spawn_basin_max_relief_m"),
    )

    nature_raw = _mapping(root.get("nature"), "nature")
    _keys(nature_raw, {"seed", "trees", "rocks", "tree_min_radius_m", "avoid_spawn_radius_m", "max_slope_deg"}, "nature")
    trees = _integer(nature_raw.get("trees"), "nature.trees")
    rocks = _integer(nature_raw.get("rocks"), "nature.rocks")
    if trees < 0 or rocks < 0:
        raise ValueError("nature.trees and nature.rocks must be nonnegative")
    nature = NatureConfig(
        seed=_integer(nature_raw.get("seed"), "nature.seed"), trees=trees, rocks=rocks,
        tree_min_radius_m=_positive(nature_raw.get("tree_min_radius_m"), "nature.tree_min_radius_m"),
        avoid_spawn_radius_m=_positive(nature_raw.get("avoid_spawn_radius_m"), "nature.avoid_spawn_radius_m"),
        max_slope_deg=_nonnegative(nature_raw.get("max_slope_deg"), "nature.max_slope_deg"),
    )

    spawn_raw = _mapping(root.get("spawn"), "spawn")
    _keys(spawn_raw, {"x_m", "y_m", "heading_deg"}, "spawn")
    spawn = SpawnConfig(
        x_m=_finite_number(spawn_raw.get("x_m"), "spawn.x_m"),
        y_m=_finite_number(spawn_raw.get("y_m"), "spawn.y_m"),
        heading_deg=_finite_number(spawn_raw.get("heading_deg"), "spawn.heading_deg"),
    )
    radius = terrain.size_m / 2.0
    if abs(spawn.x_m) > radius or abs(spawn.y_m) > radius:
        raise ValueError("spawn point must be inside map bounds")
    if abs(spawn.x_m) + terrain.spawn_basin_radius_m > radius or abs(spawn.y_m) + terrain.spawn_basin_radius_m > radius:
        raise ValueError("full spawn basin must be inside map bounds")

    return MountainMapConfig(1, "mountain_map", "world", True, terrain, nature, spawn)


def _mapping(value: object, context: str) -> Mapping[str, Any]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ValueError(f"{context} must be a mapping")
    return value


def _keys(value: Mapping[str, Any], expected: set[str], context: str) -> None:
    unknown = set(value) - expected
    missing = expected - set(value)
    if unknown:
        raise ValueError(f"{context} has unknown keys: {', '.join(sorted(unknown))}")
    if missing:
        raise ValueError(f"{context} is missing keys: {', '.join(sorted(missing))}")


def _integer(value: object, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{context} must be an integer")
    return value


def _positive_integer(value: object, context: str) -> int:
    result = _integer(value, context)
    if result <= 0:
        raise ValueError(f"{context} must be positive")
    return result


def _finite_number(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{context} must be a finite number")
    return float(value)


def _positive(value: object, context: str) -> float:
    result = _finite_number(value, context)
    if result <= 0:
        raise ValueError(f"{context} must be positive")
    return result


def _nonnegative(value: object, context: str) -> float:
    result = _finite_number(value, context)
    if result < 0:
        raise ValueError(f"{context} must be nonnegative")
    return result
