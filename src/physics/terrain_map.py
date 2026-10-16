"""Terrain coordinate queries and safe spawn poses for file-backed maps."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np

from src.physics.scenario_config import MountainMapConfig


@dataclass(frozen=True)
class SpawnPose:
    position_xyz: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    terrain_height_m: float
    launch_surface_height_m: float
    heading_deg: float


class TerrainMap:
    """Canonical image-row terrain with MuJoCo-compatible interpolation."""

    def __init__(self, heights_m: np.ndarray, size_m: float) -> None:
        values = np.asarray(heights_m, dtype=np.float32)
        if values.ndim != 2 or values.shape[0] < 2 or values.shape[1] < 2:
            raise ValueError("terrain heights must be a 2D array at least 2x2")
        if not np.isfinite(values).all():
            raise ValueError("terrain heights must be finite")
        if not math.isfinite(size_m) or size_m <= 0.0:
            raise ValueError("terrain size_m must be positive")
        self.heights_m = values
        self.size_m = float(size_m)

    @classmethod
    def from_config(
        cls,
        config: MountainMapConfig,
        asset_dir: str | Path,
    ) -> "TerrainMap":
        heights = np.load(Path(asset_dir) / "heightfield_m.npy")
        expected = (config.terrain.resolution, config.terrain.resolution)
        if heights.shape != expected:
            raise ValueError(f"terrain array shape {heights.shape} does not match {expected}")
        return cls(heights, config.terrain.size_m)

    def height_at(self, x_m: float, y_m: float) -> float:
        """Return MuJoCo triangle-interpolated height at an ENU x/y point."""
        row, col = self._fractional_indices(x_m, y_m)
        nrow, ncol = self.heights_m.shape
        r0 = min(int(math.floor(row)), nrow - 2)
        c0 = min(int(math.floor(col)), ncol - 2)
        ty, tx = row - r0, col - c0
        h00 = float(self.heights_m[r0, c0])
        h10 = float(self.heights_m[r0, c0 + 1])
        h01 = float(self.heights_m[r0 + 1, c0])
        h11 = float(self.heights_m[r0 + 1, c0 + 1])
        if tx + ty <= 1.0:
            return h00 + tx * (h10 - h00) + ty * (h01 - h00)
        return h11 + (1.0 - tx) * (h01 - h11) + (1.0 - ty) * (h10 - h11)

    def safe_spawn_pose(
        self,
        config: MountainMapConfig,
        launch_surface_height_m: float | None = None,
    ) -> SpawnPose:
        x_m, y_m = config.spawn.x_m, config.spawn.y_m
        terrain_height = self.height_at(x_m, y_m)
        surface_height = (
            terrain_height
            if launch_surface_height_m is None
            else float(launch_surface_height_m)
        )
        if not math.isfinite(surface_height):
            raise ValueError("launch surface height must be finite")
        if surface_height < terrain_height:
            raise ValueError("launch surface must not be below the terrain")
        return SpawnPose(
            position_xyz=(
                x_m,
                y_m,
                surface_height + config.terrain.spawn_clearance_m,
            ),
            quaternion_wxyz=config.spawn.quaternion_wxyz,
            terrain_height_m=terrain_height,
            launch_surface_height_m=surface_height,
            heading_deg=config.spawn.normalized_heading_deg,
        )

    def _fractional_indices(self, x_m: float, y_m: float) -> tuple[float, float]:
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            raise ValueError("terrain x/y must be finite")
        radius = self.size_m / 2.0
        if x_m < -radius or x_m > radius or y_m < -radius or y_m > radius:
            raise ValueError(
                f"terrain point ({x_m:g}, {y_m:g}) is outside "
                f"[-{radius:g}, {radius:g}] x/y bounds"
            )
        nrow, ncol = self.heights_m.shape
        col = (x_m + radius) * (ncol - 1) / self.size_m
        row = (radius - y_m) * (nrow - 1) / self.size_m
        return row, col


def physics_height_at(quad: object, x_m: float, y_m: float) -> float:
    """Ray-cast the compiled MuJoCo terrain collision surface."""
    terrain_id = int(
        mujoco.mj_name2id(quad.model, mujoco.mjtObj.mjOBJ_GEOM, "mountain_terrain")
    )
    if terrain_id < 0:
        raise ValueError("MuJoCo model has no mountain_terrain geom")
    hit_id = np.array([-1], dtype=np.int32)
    origin_z = float(quad.model.stat.extent * 2.0 + 100.0)
    distance = mujoco.mj_ray(
        quad.model,
        quad.data,
        np.array([x_m, y_m, origin_z], dtype=np.float64),
        np.array([0.0, 0.0, -1.0], dtype=np.float64),
        np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8),
        True,
        quad.body_id,
        hit_id,
    )
    if distance < 0.0 or int(hit_id[0]) != terrain_id:
        raise ValueError(f"MuJoCo terrain ray missed at ({x_m:g}, {y_m:g})")
    return origin_z - float(distance)
