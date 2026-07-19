#!/usr/bin/env python3
"""Verify the repository contract introduced by MAP-000."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAKEFILE = ROOT / "Makefile"
README = ROOT / "README.md"
MAP_PLAN = ROOT / "plans" / "mountain_terrain_map_plan.md"
BASE_PLAN = ROOT / "plans" / "ardupilot_mujoco_ros2_full_docker_plan.md"

PLACEHOLDERS = {
    "generate-map": "ERROR: generate-map is a placeholder; implemented by MAP-002",
    "up-map-viz": "ERROR: up-map-viz is a placeholder; implemented by MAP-005",
    "test-map": "ERROR: test-map is a placeholder; implemented by MAP-006",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(f"MAP-000 contract error: {message}")


def run_make(target: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["make", "--no-print-directory", target],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def verify_placeholders() -> None:
    for target, expected_message in PLACEHOLDERS.items():
        result = run_make(target)
        output = result.stdout + result.stderr
        require(result.returncode != 0, f"make {target} unexpectedly succeeded")
        require(
            expected_message in output,
            f"make {target} did not print {expected_message!r}",
        )


def verify_make_contract() -> None:
    makefile_text = MAKEFILE.read_text(encoding="utf-8")
    phony_targets: set[str] = set()
    for line in makefile_text.splitlines():
        if line.startswith(".PHONY:"):
            phony_targets.update(line.partition(":")[2].split())

    help_result = run_make("help")
    require(help_result.returncode == 0, "make help failed")
    for target in PLACEHOLDERS:
        require(target in phony_targets, f"{target} is missing from .PHONY")
        require(
            re.search(
                rf"^\s*{re.escape(target)}\s+",
                help_result.stdout,
                re.MULTILINE,
            )
            is not None,
            f"{target} is missing from make help",
        )


def verify_readme_contract() -> None:
    readme_text = README.read_text(encoding="utf-8")
    require(
        re.search(r"^## Mountain terrain map\s*$", readme_text, re.MULTILINE)
        is not None,
        "README section 'Mountain terrain map' is missing",
    )
    require(
        "[Mountain terrain map plan](plans/mountain_terrain_map_plan.md)" in readme_text,
        "README does not link to the mountain terrain map plan",
    )
    require(
        "the map feature is not implemented" in readme_text,
        "README must not imply that the mountain map already works",
    )


def unrelated_gate_changes(status_lines: list[str]) -> list[str]:
    allowed = "?? plans/mountain_terrain_map_plan.md"
    return [line for line in status_lines if line and line != allowed]


def verify_plan_contract() -> None:
    plan_text = MAP_PLAN.read_text(encoding="utf-8")
    require(
        re.search(r"\bDCK-\d{3}\b", plan_text) is None,
        "obsolete DCK ticket prefix appears in the MAP plan",
    )
    require(
        re.search(r"Do not assume .main. is the correct base\.", plan_text)
        is not None,
        "branching rule does not reject stale main as an unconditional base",
    )
    stale_main_commands = re.findall(
        r"^\s*git\s+(?:checkout|switch)\b[^\n]*(?:origin/)?main\b",
        plan_text,
        flags=re.MULTILINE | re.IGNORECASE,
    )
    require(
        not stale_main_commands,
        f"stale-main branch commands found: {stale_main_commands!r}",
    )
    require(
        "# expected: no unrelated modified/deleted files" in plan_text,
        "repository gate does not reject unrelated changes",
    )
    require(
        "# allowed before MAP-000: the canonical untracked "
        "plans/mountain_terrain_map_plan.md draft itself"
        in plan_text,
        "repository gate does not allow the canonical untracked map plan draft",
    )
    require(
        not unrelated_gate_changes(["?? plans/mountain_terrain_map_plan.md"]),
        "gate classifier rejected the canonical map plan draft",
    )
    require(
        unrelated_gate_changes(["?? unrelated.txt"]),
        "gate classifier accepted an unrelated untracked file",
    )
    require(
        unrelated_gate_changes([" M README.md"]),
        "gate classifier accepted an unrelated modified file",
    )


def verify_base_plan_preserved() -> None:
    require(BASE_PLAN.is_file(), "active base Docker/RViz plan was deleted")
    base_plan_text = BASE_PLAN.read_text(encoding="utf-8")
    require(
        base_plan_text.startswith("# Full Docker Setup Plan:"),
        "active base plan does not contain the completed Docker/RViz plan",
    )
    require(
        len(base_plan_text.splitlines()) > 500,
        "active base plan is unexpectedly incomplete",
    )


def main() -> None:
    verify_placeholders()
    verify_make_contract()
    verify_readme_contract()
    verify_plan_contract()
    verify_base_plan_preserved()
    print("PASS: MAP-000 map plan and intentionally failing placeholders are correct")


if __name__ == "__main__":
    main()
