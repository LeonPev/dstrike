#!/usr/bin/env python3
"""Compile one scenario through the canonical loader and explicit MjVfs."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.scenarios import build_scenario_model_xml, scenario_from_name


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default=None)
    args = parser.parse_args()
    scenario = scenario_from_name(args.scenario)
    model_xml = build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario)
    quad = MuJoCoQuad(
        model_xml=model_xml,
        model_source=f"{DEFAULT_MODEL_PATH} + {scenario.xml_path}",
        vfs_files=scenario.vfs_files,
    )
    print(
        f"scenario={scenario.name} ngeom={quad.model.ngeom} "
        f"nhfield={quad.model.nhfield}"
    )
    print(f"PASS: scenario {scenario.name} compiled through canonical loader")


if __name__ == "__main__":
    main()
