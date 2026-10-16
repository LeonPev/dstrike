"""Strict configuration contract for file-backed map scenarios."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Protocol

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


@dataclass(frozen=True)
class MapReferenceConfig:
    name: str


@dataclass(frozen=True)
class TargetDroneConfig:
    name: str
    start_xy_m: tuple[float, float]
    heading_deg: float
    speed_mps: float
    altitude_agl_m: float
    route_length_m: float
    route_behavior: str

    @property
    def normalized_heading_deg(self) -> float:
        return self.heading_deg % 360.0


@dataclass(frozen=True)
class ChaserDroneConfig:
    name: str
    start_xy_m: tuple[float, float]
    heading_deg: float
    initial_agl_m: float
    takeoff_agl_m: float
    formation_offset_target_body_m: tuple[float, float, float]

    @property
    def normalized_heading_deg(self) -> float:
        return self.heading_deg % 360.0


@dataclass(frozen=True)
class SafetyConfig:
    mission_timeout_s: float
    formation_error_threshold_m: float
    formation_hold_required_s: float
    min_terrain_clearance_m: float
    min_drone_separation_m: float
    max_chaser_speed_mps: float
    geofence_margin_m: float


@dataclass(frozen=True)
class VisualizationConfig:
    camera_rate_hz: float
    marker_rate_hz: float


@dataclass(frozen=True)
class TwoDroneScenarioConfig:
    name: str
    frame_id: str
    map: MapReferenceConfig
    target_drone: TargetDroneConfig
    chaser_drone: ChaserDroneConfig
    safety: SafetyConfig
    visualization: VisualizationConfig

    @property
    def route_duration_s(self) -> float:
        return self.target_drone.route_length_m / self.target_drone.speed_mps

    @property
    def target_route_end_xy_m(self) -> tuple[float, float]:
        yaw = math.radians(self.target_drone.normalized_heading_deg)
        return (
            self.target_drone.start_xy_m[0]
            + self.target_drone.route_length_m * math.cos(yaw),
            self.target_drone.start_xy_m[1]
            + self.target_drone.route_length_m * math.sin(yaw),
        )


class TerrainHeightProvider(Protocol):
    size_m: float

    def height_at(self, x_m: float, y_m: float) -> float: ...


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


def load_two_drone_config(
    path: str | Path,
    *,
    map_config_path: str | Path | None = None,
    map_asset_dir: str | Path | None = None,
) -> TwoDroneScenarioConfig:
    """Load SCN-001 and validate it against the completed mountain map."""
    scenario_path = Path(path)
    with scenario_path.open("r", encoding="utf-8") as config_file:
        raw = yaml.safe_load(config_file)

    root = scenario_path.resolve().parents[2]
    assets = Path(map_asset_dir) if map_asset_dir else root / "models/scenarios/mountain_map"
    map_path = Path(map_config_path) if map_config_path else assets / "scenario.yaml"
    mountain_config = load_mountain_map_config(map_path)
    # Local import avoids a module cycle: terrain_map uses MountainMapConfig.
    from src.physics.terrain_map import TerrainMap

    terrain = TerrainMap.from_config(mountain_config, assets)
    return validate_two_drone_config(raw, mountain_config=mountain_config, terrain=terrain)


def validate_two_drone_config(
    raw: object,
    *,
    mountain_config: MountainMapConfig,
    terrain: TerrainHeightProvider,
) -> TwoDroneScenarioConfig:
    root = _mapping(raw, "scenario")
    _keys(root, {"name", "frame_id", "map", "target_drone", "chaser_drone", "safety", "visualization"}, "scenario")
    if root.get("name") != "mountain_two_drones":
        raise ValueError("name must equal mountain_two_drones")
    if root.get("frame_id") != "world":
        raise ValueError("frame_id must equal world")

    map_raw = _mapping(root.get("map"), "map")
    _keys(map_raw, {"name"}, "map")
    if map_raw.get("name") != "mountain_map" or mountain_config.name != "mountain_map":
        raise ValueError("map.name must equal mountain_map")

    target_raw = _mapping(root.get("target_drone"), "target_drone")
    _keys(target_raw, {"name", "start_xy_m", "heading_deg", "speed_mps", "altitude_agl_m", "route_length_m", "route_behavior"}, "target_drone")
    if target_raw.get("name") != "target":
        raise ValueError("target_drone.name must equal target")
    if target_raw.get("route_behavior") != "ping_pong":
        raise ValueError("target_drone.route_behavior must equal ping_pong")
    target = TargetDroneConfig(
        name="target",
        start_xy_m=_vector(target_raw.get("start_xy_m"), 2, "target_drone.start_xy_m"),
        heading_deg=_finite_number(target_raw.get("heading_deg"), "target_drone.heading_deg") % 360.0,
        speed_mps=_positive(target_raw.get("speed_mps"), "target_drone.speed_mps"),
        altitude_agl_m=_nonnegative(target_raw.get("altitude_agl_m"), "target_drone.altitude_agl_m"),
        route_length_m=_positive(target_raw.get("route_length_m"), "target_drone.route_length_m"),
        route_behavior="ping_pong",
    )

    chaser_raw = _mapping(root.get("chaser_drone"), "chaser_drone")
    _keys(chaser_raw, {"name", "start_xy_m", "heading_deg", "initial_agl_m", "takeoff_agl_m", "formation_offset_target_body_m"}, "chaser_drone")
    if chaser_raw.get("name") != "chaser":
        raise ValueError("chaser_drone.name must equal chaser")
    chaser = ChaserDroneConfig(
        name="chaser",
        start_xy_m=_vector(chaser_raw.get("start_xy_m"), 2, "chaser_drone.start_xy_m"),
        heading_deg=_finite_number(chaser_raw.get("heading_deg"), "chaser_drone.heading_deg") % 360.0,
        initial_agl_m=_nonnegative(chaser_raw.get("initial_agl_m"), "chaser_drone.initial_agl_m"),
        takeoff_agl_m=_nonnegative(chaser_raw.get("takeoff_agl_m"), "chaser_drone.takeoff_agl_m"),
        formation_offset_target_body_m=_vector(chaser_raw.get("formation_offset_target_body_m"), 3, "chaser_drone.formation_offset_target_body_m"),
    )
    if chaser.takeoff_agl_m <= chaser.initial_agl_m:
        raise ValueError("chaser_drone.takeoff_agl_m must be greater than initial_agl_m")

    safety_raw = _mapping(root.get("safety"), "safety")
    _keys(safety_raw, {"mission_timeout_s", "formation_error_threshold_m", "formation_hold_required_s", "min_terrain_clearance_m", "min_drone_separation_m", "max_chaser_speed_mps", "geofence_margin_m"}, "safety")
    safety = SafetyConfig(
        mission_timeout_s=_positive(safety_raw.get("mission_timeout_s"), "safety.mission_timeout_s"),
        formation_error_threshold_m=_positive(safety_raw.get("formation_error_threshold_m"), "safety.formation_error_threshold_m"),
        formation_hold_required_s=_positive(safety_raw.get("formation_hold_required_s"), "safety.formation_hold_required_s"),
        min_terrain_clearance_m=_positive(safety_raw.get("min_terrain_clearance_m"), "safety.min_terrain_clearance_m"),
        min_drone_separation_m=_positive(safety_raw.get("min_drone_separation_m"), "safety.min_drone_separation_m"),
        max_chaser_speed_mps=_positive(safety_raw.get("max_chaser_speed_mps"), "safety.max_chaser_speed_mps"),
        geofence_margin_m=_nonnegative(safety_raw.get("geofence_margin_m"), "safety.geofence_margin_m"),
    )

    visualization_raw = _mapping(root.get("visualization"), "visualization")
    _keys(visualization_raw, {"camera_rate_hz", "marker_rate_hz"}, "visualization")
    visualization = VisualizationConfig(
        camera_rate_hz=_positive(visualization_raw.get("camera_rate_hz"), "visualization.camera_rate_hz"),
        marker_rate_hz=_positive(visualization_raw.get("marker_rate_hz"), "visualization.marker_rate_hz"),
    )
    config = TwoDroneScenarioConfig("mountain_two_drones", "world", MapReferenceConfig("mountain_map"), target, chaser, safety, visualization)

    if config.route_duration_s < safety.mission_timeout_s + 10.0:
        raise ValueError("target route duration must be at least mission_timeout_s + 10.0")
    if target.altitude_agl_m < safety.min_terrain_clearance_m:
        raise ValueError("target route must maintain minimum terrain clearance")
    if math.dist((0.0, 0.0, 0.0), chaser.formation_offset_target_body_m) < safety.min_drone_separation_m:
        raise ValueError("formation offset norm must be at least min_drone_separation_m")

    radius = mountain_config.terrain.size_m / 2.0
    usable_radius = radius - safety.geofence_margin_m
    if usable_radius < 0.0:
        raise ValueError("safety.geofence_margin_m exceeds mountain map bounds")
    route_end = config.target_route_end_xy_m
    for label, point in (("target spawn", target.start_xy_m), ("target route end", route_end), ("chaser spawn", chaser.start_xy_m)):
        if abs(point[0]) > usable_radius or abs(point[1]) > usable_radius:
            raise ValueError(f"{label} must remain inside the MAP geofence margin")
        terrain.height_at(*point)
    # Exercise the complete line against the canonical terrain query. Since the
    # route altitude is terrain-relative, every sample has the configured AGL.
    samples = max(2, int(math.ceil(target.route_length_m / 2.0)) + 1)
    for index in range(samples):
        fraction = index / (samples - 1)
        x_m = target.start_xy_m[0] + fraction * (route_end[0] - target.start_xy_m[0])
        y_m = target.start_xy_m[1] + fraction * (route_end[1] - target.start_xy_m[1])
        terrain_height = terrain.height_at(x_m, y_m)
        if not math.isfinite(terrain_height + target.altitude_agl_m):
            raise ValueError("target route produced a non-finite world altitude")
    return config


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


def _vector(value: object, length: int, context: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{context} must contain exactly {length} numbers")
    return tuple(_finite_number(item, f"{context}[{index}]") for index, item in enumerate(value))
