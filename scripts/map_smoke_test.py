#!/usr/bin/env python3
"""Fast in-container scenario and MuJoCo compilation checks for MAP-006."""

from __future__ import annotations

from pathlib import Path
import sys

import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.scenarios import build_scenario_model_xml, scenario_from_name


def geom_id(model: mujoco.MjModel, name: str) -> int:
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))


def compile_scenario(name: str) -> MuJoCoQuad:
    scenario = scenario_from_name(name)
    return MuJoCoQuad(
        model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario),
        model_source=f"{DEFAULT_MODEL_PATH} + {scenario.xml_path}",
        vfs_files=scenario.vfs_files,
    )


def main() -> None:
    default = scenario_from_name("default")
    assert default.name == "landing_pad"
    default_quad = compile_scenario("default")
    assert geom_id(default_quad.model, "visual_ground") >= 0
    assert geom_id(default_quad.model, "mountain_terrain") == -1
    assert default_quad.model.nhfield == 0

    mountain = scenario_from_name("mountain_map")
    assert mountain.config is not None
    assert set(mountain.vfs_files) == {"mountain_map/heightfield.bin"}
    mountain_quad = compile_scenario("mountain_map")
    assert geom_id(mountain_quad.model, "visual_ground") == -1
    assert geom_id(mountain_quad.model, "mountain_terrain") >= 0
    assert mountain_quad.model.nhfield == 1

    print("MAP-006 scenario loader and MuJoCo compilation smoke checks passed")


if __name__ == "__main__":
    main()
