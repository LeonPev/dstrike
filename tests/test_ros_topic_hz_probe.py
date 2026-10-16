from __future__ import annotations

import importlib.util
from pathlib import Path


def load_ros_topic_hz_probe():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "ros_topic_hz_probe.py"
    spec = importlib.util.spec_from_file_location("ros_topic_hz_probe", script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_calculate_rate_hz_uses_first_and_last_sample_window() -> None:
    module = load_ros_topic_hz_probe()

    assert module.calculate_rate_hz([]) == 0.0
    assert module.calculate_rate_hz([1.0]) == 0.0
    assert module.calculate_rate_hz([1.0, 1.5, 2.0]) == 2.0


def test_calculate_jitter_s_handles_short_and_regular_samples() -> None:
    module = load_ros_topic_hz_probe()

    assert module.calculate_jitter_s([1.0, 2.0]) == 0.0
    assert module.calculate_jitter_s([1.0, 1.5, 2.0, 2.5]) == 0.0
