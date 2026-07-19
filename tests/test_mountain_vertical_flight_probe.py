from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import pytest


def load_probe():
    script_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "mountain_vertical_flight_probe.py"
    )
    module_name = "mountain_vertical_flight_probe"
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def test_quaternion_tilt_ignores_yaw_and_rejects_non_finite_values() -> None:
    probe = load_probe()
    half_sqrt = math.sqrt(0.5)

    assert probe.quaternion_tilt_deg(0.0, 0.0, half_sqrt, half_sqrt) == 0.0
    assert probe.quaternion_tilt_deg(half_sqrt, 0.0, 0.0, half_sqrt) == pytest.approx(
        90.0
    )
    assert math.isnan(probe.quaternion_tilt_deg(math.nan, 0.0, 0.0, 1.0))


def test_window_stats_measure_relative_drift_and_vertical_stability() -> None:
    probe = load_probe()
    samples = [
        probe.GroundTruthSample(0.0, 0.0, 0.0, 10.0, 0.1, 1.0),
        probe.GroundTruthSample(1.0, 0.3, 0.0, 10.2, -0.2, 2.0),
        probe.GroundTruthSample(2.0, 0.4, 0.0, 10.1, 0.0, 3.0),
    ]

    stats = probe.window_stats(samples, (0.0, 0.0))

    assert stats.count == 3
    assert stats.duration_s == 2.0
    assert stats.median_x_m == 0.3
    assert stats.median_z_m == 10.1
    assert stats.z_span_m == pytest.approx(0.2)
    assert stats.max_abs_vz_m_s == 0.2
    assert stats.max_tilt_deg == 3.0
    assert stats.max_xy_drift_m == 0.4


def test_window_stats_reject_non_finite_ground_truth() -> None:
    probe = load_probe()
    sample = probe.GroundTruthSample(0.0, 0.0, 0.0, math.nan, 0.0, 0.0)

    with pytest.raises(ValueError, match="must be finite"):
        probe.window_stats([sample], (0.0, 0.0))


def test_local_position_readiness_requires_explicit_bridge_flag() -> None:
    probe = load_probe()

    assert not probe.heartbeat_has_local_position_ned(None)
    assert not probe.heartbeat_has_local_position_ned({"armed": False})
    assert not probe.heartbeat_has_local_position_ned(
        {"has_local_position_ned": False}
    )
    assert probe.heartbeat_has_local_position_ned(
        {"has_local_position_ned": True}
    )


def test_landed_window_requires_fresh_level_samples_inside_pad_margin() -> None:
    probe = load_probe()
    ground_xyz = (-90.0, -40.0, 12.75)
    samples = [
        probe.GroundTruthSample(10.0, -90.0, -40.0, 12.75, 0.01, 0.1),
        probe.GroundTruthSample(11.5, -89.9, -40.0, 12.76, 0.0, 0.2),
    ]
    kwargs = {
        "samples": samples,
        "ground_xyz": ground_xyz,
        "now_s": 11.6,
        "minimum_duration_s": 1.2,
        "telemetry_max_age_s": 1.0,
        "z_tolerance_m": 0.15,
        "max_abs_vz_m_s": 0.2,
        "max_xy_error_m": 1.3,
        "max_tilt_deg": 5.0,
    }

    assert probe.landed_window_is_stable(**kwargs)
    assert not probe.landed_window_is_stable(**{**kwargs, "now_s": 13.0})

    outside_safety_margin = list(samples)
    outside_safety_margin[-1] = probe.GroundTruthSample(
        11.5, -88.6, -40.0, 12.76, 0.0, 0.2
    )
    assert not probe.landed_window_is_stable(
        **{**kwargs, "samples": outside_safety_margin}
    )

    tipped = list(samples)
    tipped[-1] = probe.GroundTruthSample(
        11.5, -89.9, -40.0, 12.76, 0.0, 15.0
    )
    assert not probe.landed_window_is_stable(
        **{**kwargs, "samples": tipped}
    )


def test_probe_refuses_to_validate_a_non_mountain_stack(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    probe = load_probe()
    monkeypatch.setenv("SCENARIO", "landing_pad")
    monkeypatch.setattr(sys, "argv", ["mountain_vertical_flight_probe.py"])

    assert probe.main() == 2
