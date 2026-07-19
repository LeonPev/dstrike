#!/usr/bin/env python3
"""Bounded MAP-003 VFS, row-order, ray, and real-contact verifier."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.scenarios import build_scenario_model_xml, scenario_from_name


TOLERANCE_M = 0.10


def triangle_height(values: np.ndarray, x_m: float, y_m: float, size_m: float, max_height_m: float) -> float:
    """Match MuJoCo's two-triangle interpolation on the canonical image array."""
    nrow, ncol = values.shape
    col = (x_m + size_m / 2.0) * (ncol - 1) / size_m
    row = (size_m / 2.0 - y_m) * (nrow - 1) / size_m
    col = min(max(col, 0.0), ncol - 1.0)
    row = min(max(row, 0.0), nrow - 1.0)
    c0, r0 = min(int(math.floor(col)), ncol - 2), min(int(math.floor(row)), nrow - 2)
    tx, ty = col - c0, row - r0
    h00, h10 = float(values[r0, c0]), float(values[r0, c0 + 1])
    h01, h11 = float(values[r0 + 1, c0]), float(values[r0 + 1, c0 + 1])
    if tx + ty <= 1.0:
        height_norm = h00 + tx * (h10 - h00) + ty * (h01 - h00)
    else:
        height_norm = h11 + (1.0 - tx) * (h01 - h11) + (1.0 - ty) * (h10 - h11)
    return height_norm * max_height_m


def geom_id(model: mujoco.MjModel, name: str) -> int:
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))


def ray_height(quad: MuJoCoQuad, terrain_id: int, x_m: float, y_m: float) -> float:
    hit_id = np.array([-1], dtype=np.int32)
    origin_z = 100.0
    distance = mujoco.mj_ray(
        quad.model,
        quad.data,
        np.array([x_m, y_m, origin_z], dtype=np.float64),
        np.array([0.0, 0.0, -1.0], dtype=np.float64),
        None,
        True,
        quad.body_id,
        hit_id,
    )
    assert distance >= 0.0, f"ray missed terrain at ({x_m}, {y_m})"
    assert int(hit_id[0]) == terrain_id, (x_m, y_m, int(hit_id[0]), terrain_id)
    return origin_z - float(distance)


def add_contact_probe(model_xml: str, x_m: float, y_m: float, start_z: float) -> str:
    root = ET.fromstring(model_xml)
    worldbody = root.find("worldbody")
    assert worldbody is not None
    body = ET.SubElement(worldbody, "body", name="terrain_contact_probe", pos=f"{x_m:g} {y_m:g} {start_z:g}")
    ET.SubElement(body, "joint", name="probe_slide", type="slide", axis="0 0 1")
    ET.SubElement(
        body,
        "geom",
        name="terrain_contact_ball",
        type="sphere",
        size="0.2",
        mass="0.1",
        contype="1",
        conaffinity="1",
        friction="1 0.005 0.0001",
    )
    return ET.tostring(root, encoding="unicode")


def main() -> None:
    scenario = scenario_from_name("mountain_map")
    assert scenario.config is not None
    config = scenario.config
    terrain = config.terrain
    model_xml = build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario)
    quad = MuJoCoQuad(model_xml=model_xml, vfs_files=scenario.vfs_files)
    terrain_id = geom_id(quad.model, "mountain_terrain")
    assert terrain_id >= 0
    assert geom_id(quad.model, "visual_ground") == -1

    hfield_id = int(quad.model.geom_dataid[terrain_id])
    assert tuple(quad.model.hfield_size[hfield_id]) == (
        terrain.size_m / 2.0,
        terrain.size_m / 2.0,
        terrain.max_height_m,
        terrain.base_depth_m,
    )
    address = int(quad.model.hfield_adr[hfield_id])
    nrow = int(quad.model.hfield_nrow[hfield_id])
    ncol = int(quad.model.hfield_ncol[hfield_id])
    compiled = np.asarray(quad.model.hfield_data[address : address + nrow * ncol]).reshape(nrow, ncol)
    canonical = np.load(ROOT / "models/scenarios/mountain_map/heightfield_norm.npy")
    expected_compiled = canonical[::-1, :]
    assert np.array_equal(compiled, expected_compiled)

    peak_rc = tuple(int(value) for value in np.unravel_index(np.argmax(canonical), canonical.shape))
    valley_rc = tuple(int(value) for value in np.unravel_index(np.argmin(canonical), canonical.shape))
    sample_rc = [(0, 0), (0, ncol - 1), (nrow - 1, 0), (nrow - 1, ncol - 1), (nrow // 2, ncol // 2), peak_rc, valley_rc, (17, 31), (44, 79), (96, 23)]
    for row, col in sample_rc:
        assert compiled[nrow - 1 - row, col] == canonical[row, col]
    spawn_col = round((config.spawn.x_m + terrain.size_m / 2.0) * (ncol - 1) / terrain.size_m)
    spawn_row = round((config.spawn.y_m + terrain.size_m / 2.0) * (nrow - 1) / terrain.size_m)
    assert compiled[nrow - 1 - spawn_row, spawn_col] == canonical[spawn_row, spawn_col]

    spacing = terrain.size_m / (nrow - 1)
    def world(row: float, col: float) -> tuple[float, float]:
        return (-terrain.size_m / 2.0 + col * spacing, terrain.size_m / 2.0 - row * spacing)

    ray_points = [
        world(0.25, 0.25), world(0.25, ncol - 1.25),
        world(nrow - 1.25, 0.25), world(nrow - 1.25, ncol - 1.25),
        (0.0, 0.0), (config.spawn.x_m, config.spawn.y_m),
        world(*peak_rc), world(*valley_rc),
        world(17.2, 31.3), world(44.6, 79.2), world(96.4, 23.7),
    ]
    # Find an asymmetric cell and sample both sides of its diagonal.
    chosen = None
    for row in range(1, nrow - 2):
        for col in range(1, ncol - 2):
            cross = canonical[row, col] + canonical[row + 1, col + 1] - canonical[row, col + 1] - canonical[row + 1, col]
            if abs(float(cross)) > 0.003:
                chosen = (row, col)
                break
        if chosen:
            break
    assert chosen is not None
    row, col = chosen
    ray_points.extend((world(row + 0.2, col + 0.6), world(row + 0.8, col + 0.4)))
    for x_m, y_m in ray_points:
        expected = triangle_height(canonical, x_m, y_m, terrain.size_m, terrain.max_height_m)
        actual = ray_height(quad, terrain_id, x_m, y_m)
        assert abs(actual - expected) <= TOLERANCE_M, (x_m, y_m, expected, actual)

    default = scenario_from_name("default")
    default_quad = MuJoCoQuad(
        model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, default),
        vfs_files=default.vfs_files,
    )
    assert geom_id(default_quad.model, "visual_ground") >= 0
    assert geom_id(default_quad.model, "mountain_terrain") == -1

    contact_x, contact_y = world(64.0, 64.0)
    contact_height = triangle_height(canonical, contact_x, contact_y, terrain.size_m, terrain.max_height_m)
    contact_xml = add_contact_probe(model_xml, contact_x, contact_y, contact_height + 2.0)
    contact_quad = MuJoCoQuad(model_xml=contact_xml, vfs_files=scenario.vfs_files)
    contact_terrain_id = geom_id(contact_quad.model, "mountain_terrain")
    probe_geom_id = geom_id(contact_quad.model, "terrain_contact_ball")
    for _ in range(4000):
        mujoco.mj_step(contact_quad.model, contact_quad.data)
    pairs = {
        frozenset((int(contact_quad.data.contact[index].geom1), int(contact_quad.data.contact[index].geom2)))
        for index in range(contact_quad.data.ncon)
    }
    assert contact_quad.data.ncon > 0
    assert frozenset((contact_terrain_id, probe_geom_id)) in pairs
    probe_body_id = mujoco.mj_name2id(contact_quad.model, mujoco.mjtObj.mjOBJ_BODY, "terrain_contact_probe")
    probe_bottom = float(contact_quad.data.xpos[probe_body_id, 2]) - 0.2
    assert abs(probe_bottom - contact_height) <= TOLERANCE_M, (probe_bottom, contact_height)

    print("PASS: MAP-003 MuJoCo VFS heightfield, row order, ground-plane policy, and terrain collision are verified")


if __name__ == "__main__":
    main()
