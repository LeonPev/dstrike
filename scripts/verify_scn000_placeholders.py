#!/usr/bin/env python3
"""Verify the SCN-000 plan link and intentionally failing placeholders."""

from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
PLAN_PATH = "plans/two_drone_mountain_scenario_plan.md"


def placeholder_output(target: str, owner: str) -> str:
    result = subprocess.run(
        ["make", "--no-print-directory", target],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode != 0, f"{target} must fail intentionally"
    assert owner in output, f"{target} must identify {owner} as its owner"
    assert "No rule to make target" not in output
    assert "command not found" not in output
    return output


def main() -> None:
    assert (ROOT / PLAN_PATH).is_file()
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert PLAN_PATH in readme

    makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
    phony_line = next(line for line in makefile.splitlines() if line.startswith(".PHONY:"))
    for target in ("up-two-drone-viz", "test-e2e-two-drone"):
        assert target in phony_line
        assert f"{target}:" in makefile

    placeholder_output("up-two-drone-viz", "SCN-007")
    placeholder_output("test-e2e-two-drone", "SCN-009")

    assert not (ROOT / "config/scenarios/mountain_two_drones.yaml").exists()
    assert not (ROOT / "models/scenarios/mountain_two_drones.xml").exists()

    print("PASS: SCN-000 two-drone plan and placeholder targets are present")


if __name__ == "__main__":
    main()
