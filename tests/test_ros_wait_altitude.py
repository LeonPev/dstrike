from __future__ import annotations

import importlib.util
from pathlib import Path


def load_ros_wait_altitude():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "ros_wait_altitude.py"
    spec = importlib.util.spec_from_file_location("ros_wait_altitude", script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_altitude_delta_uses_ros_enu_z_up_difference() -> None:
    module = load_ros_wait_altitude()

    assert module.altitude_delta(-0.25, 0.75) == 1.0


def test_altitude_summary_prints_start_latest_and_delta() -> None:
    module = load_ros_wait_altitude()

    assert module.altitude_summary(0.0, 0.75) == (
        "starting_altitude_m=0.000 latest_altitude_m=0.750 delta_m=0.750"
    )
