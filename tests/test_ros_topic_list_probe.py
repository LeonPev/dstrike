from __future__ import annotations

import importlib.util
from pathlib import Path


def load_ros_topic_list_probe():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "ros_topic_list_probe.py"
    spec = importlib.util.spec_from_file_location("ros_topic_list_probe", script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_pass_line_is_dfd_015_specific() -> None:
    module = load_ros_topic_list_probe()

    assert module.PASS_LINE == "PASS: DFD-015 topic list passed"
