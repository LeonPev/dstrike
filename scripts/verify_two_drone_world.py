#!/usr/bin/env python3
"""Bounded SCN-002 world-composition verifier."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.scenarios import build_scenario_model_xml, scenario_from_name
from src.physics.terrain_map import TerrainMap


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    return int(mujoco.mj_name2id(model, kind, name))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="mountain_two_drones")
    args = parser.parse_args()
    assert args.scenario.replace("-", "_") == "mountain_two_drones"

    scenario = scenario_from_name(args.scenario)
    assert scenario.config is not None
    assert scenario.two_drone_config is not None
    model_xml = build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario)
    quad = MuJoCoQuad(model_xml=model_xml, vfs_files=scenario.vfs_files)
    terrain = TerrainMap.from_config(
        scenario.config,
        ROOT / "models/scenarios/mountain_map",
    )

    config = scenario.two_drone_config
    for name, xy_m, agl_m in (
        ("target", config.target_drone.start_xy_m, config.target_drone.altitude_agl_m),
        ("chaser", config.chaser_drone.start_xy_m, config.chaser_drone.initial_agl_m),
    ):
        body_id = object_id(quad.model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert body_id >= 0
        actual_z = float(quad.model.body_pos[body_id, 2])
        expected_z = terrain.height_at(*xy_m) + agl_m
        assert abs(actual_z - expected_z) <= 1e-6
        print(f"{name} spawn xyz_m=({xy_m[0]:.3f}, {xy_m[1]:.3f}, {actual_z:.3f})")

    target_id = object_id(quad.model, mujoco.mjtObj.mjOBJ_BODY, "target")
    target_geoms = [i for i in range(quad.model.ngeom) if int(quad.model.geom_bodyid[i]) == target_id]
    assert target_geoms
    assert all(int(quad.model.geom_contype[i]) == 0 for i in target_geoms)
    assert all(int(quad.model.geom_conaffinity[i]) == 0 for i in target_geoms)
    assert object_id(quad.model, mujoco.mjtObj.mjOBJ_BODY, "chaser") == quad.body_id
    terrain_id = object_id(quad.model, mujoco.mjtObj.mjOBJ_GEOM, "mountain_terrain")
    assert terrain_id >= 0

    public_names: list[str] = []
    for kind, count in (
        (mujoco.mjtObj.mjOBJ_BODY, quad.model.nbody),
        (mujoco.mjtObj.mjOBJ_GEOM, quad.model.ngeom),
        (mujoco.mjtObj.mjOBJ_SITE, quad.model.nsite),
        (mujoco.mjtObj.mjOBJ_CAMERA, quad.model.ncam),
        (mujoco.mjtObj.mjOBJ_SENSOR, quad.model.nsensor),
        (mujoco.mjtObj.mjOBJ_ACTUATOR, quad.model.nu),
    ):
        names = [mujoco.mj_id2name(quad.model, kind, index) for index in range(count)]
        named = [name for name in names if name is not None]
        assert len(named) == len(set(named))
        public_names.extend(name for name in named if name.startswith(("target", "chaser")))
    assert len(public_names) == len(set(public_names))
    assert object_id(quad.model, mujoco.mjtObj.mjOBJ_CAMERA, "target_front_cam") >= 0
    assert object_id(quad.model, mujoco.mjtObj.mjOBJ_CAMERA, "chaser_front_cam") >= 0

    chaser_id = quad.body_id
    quad.reset((0.0, 0.0, terrain.height_at(0.0, 0.0) + 0.5))
    touched_terrain = False
    for _ in range(2000):
        mujoco.mj_step(quad.model, quad.data)
        for index in range(quad.data.ncon):
            pair = {
                int(quad.data.contact[index].geom1),
                int(quad.data.contact[index].geom2),
            }
            if terrain_id in pair and any(
                int(quad.model.geom_bodyid[geom_id]) == chaser_id for geom_id in pair
            ):
                touched_terrain = True
                break
        if touched_terrain:
            break
    assert touched_terrain

    for regression in ("landing_pad", "mountain_map"):
        candidate = scenario_from_name(regression)
        MuJoCoQuad(
            model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, candidate),
            vfs_files=candidate.vfs_files,
        )
        print(f"regression scenario compiled: {regression}")

    print("PASS: SCN-002 two-drone MuJoCo world composes with the existing mountain terrain")


if __name__ == "__main__":
    main()
