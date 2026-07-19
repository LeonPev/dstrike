"""Deterministic procedural terrain generation for the mountain map."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from src.physics.scenario_config import MountainMapConfig


@dataclass(frozen=True)
class TerrainAssets:
    height_norm: np.ndarray
    height_m: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    slope_deg: np.ndarray


def generate_terrain(config: MountainMapConfig) -> TerrainAssets:
    terrain = config.terrain
    rng = np.random.Generator(np.random.PCG64(terrain.seed))
    x_axis = np.linspace(-terrain.size_m / 2.0, terrain.size_m / 2.0, terrain.resolution)
    # Canonical image row order is north-to-south: row 0 is +Y.
    y_axis = np.linspace(terrain.size_m / 2.0, -terrain.size_m / 2.0, terrain.resolution)
    x_grid, y_grid = np.meshgrid(x_axis, y_axis)

    raw = np.zeros_like(x_grid)
    peaks = (
        (-92.0, 82.0, 44.0, 65.0),
        (72.0, 67.0, 38.0, 55.0),
        (83.0, -66.0, 35.0, 48.0),
        (-28.0, -93.0, 29.0, 43.0),
        (8.0, 16.0, 22.0, 72.0),
    )
    for peak_x, peak_y, amplitude, width in peaks:
        raw += amplitude * np.exp(-((x_grid - peak_x) ** 2 + (y_grid - peak_y) ** 2) / (2.0 * width**2))

    # A handful of long waves gives repeatable, low-frequency variation without
    # introducing a SciPy dependency or any non-local input data.
    for _ in range(7):
        wavelength = rng.uniform(65.0, 170.0)
        angle = rng.uniform(0.0, 2.0 * math.pi)
        phase = rng.uniform(0.0, 2.0 * math.pi)
        amplitude = rng.uniform(0.8, 3.2)
        projected = x_grid * math.cos(angle) + y_grid * math.sin(angle)
        raw += amplitude * np.sin(2.0 * math.pi * projected / wavelength + phase)

    spacing = terrain.size_m / (terrain.resolution - 1)
    basin_distance = np.hypot(x_grid - config.spawn.x_m, y_grid - config.spawn.y_m)
    flat_radius = terrain.spawn_basin_radius_m + spacing * 1.5
    blend_width = max(12.0, spacing * 5.0)
    basin_level = float(np.median(raw[basin_distance <= terrain.spawn_basin_radius_m]))
    blend = np.clip((basin_distance - flat_radius) / blend_width, 0.0, 1.0)
    blend = blend * blend * (3.0 - 2.0 * blend)
    raw = basin_level * (1.0 - blend) + raw * blend

    raw -= raw.min()
    raw /= raw.max()
    raw = np.round(raw, terrain.quantization_decimals)

    # These deliberately asymmetric corners are outside the basin and make an
    # accidental canonical-row/MuJoCo-row mixup immediately detectable.
    raw[0, 0], raw[0, -1], raw[-1, 0], raw[-1, -1] = 0.0, 0.173, 0.619, 1.0
    height_norm = np.asarray(raw, dtype="<f4")
    height_m = np.asarray(
        np.round(height_norm.astype(np.float64) * terrain.max_height_m, terrain.quantization_decimals),
        dtype="<f4",
    )
    dy, dx = np.gradient(height_m.astype(np.float64), spacing, spacing)
    slope_deg = np.degrees(np.arctan(np.hypot(dx, dy)))
    return TerrainAssets(height_norm, height_m, x_grid, y_grid, slope_deg)


def bilinear_sample(values: np.ndarray, x_m: float, y_m: float, size_m: float) -> float:
    """Sample an image-row-order terrain array at world x/y coordinates."""
    resolution = values.shape[0]
    col = (x_m + size_m / 2.0) * (resolution - 1) / size_m
    row = (size_m / 2.0 - y_m) * (resolution - 1) / size_m
    c0, r0 = int(math.floor(col)), int(math.floor(row))
    c1, r1 = min(c0 + 1, resolution - 1), min(r0 + 1, resolution - 1)
    tx, ty = col - c0, row - r0
    return float(
        values[r0, c0] * (1.0 - tx) * (1.0 - ty)
        + values[r0, c1] * tx * (1.0 - ty)
        + values[r1, c0] * (1.0 - tx) * ty
        + values[r1, c1] * tx * ty
    )
