from __future__ import annotations

import importlib.util
from pathlib import Path


def load_e2e_takeoff():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "e2e_takeoff.py"
    spec = importlib.util.spec_from_file_location("e2e_takeoff", script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_heartbeat_payload_requires_mavlink_identity_fields() -> None:
    module = load_e2e_takeoff()

    heartbeat = module.parse_heartbeat_payload(
        '{"sysid": 1, "compid": 1, "mode": "GUIDED", "armed": true}'
    )

    assert heartbeat["sysid"] == 1
    assert heartbeat["compid"] == 1
    assert heartbeat["armed"] is True
    assert module.parse_heartbeat_payload("not-json") is None
    assert module.parse_heartbeat_payload('{"mode": "GUIDED"}') is None


def test_altitude_threshold_must_be_exceeded() -> None:
    module = load_e2e_takeoff()

    assert not module.altitude_threshold_reached(None, 1.0)
    assert not module.altitude_threshold_reached(1.0, 1.0)
    assert module.altitude_threshold_reached(1.01, 1.0)


def test_altitude_summary_includes_start_latest_delta_and_threshold() -> None:
    module = load_e2e_takeoff()

    assert module.altitude_summary(0.1, 1.25, 1.0) == (
        "starting_altitude_m=0.100 "
        "latest_altitude_m=1.250 "
        "delta_m=1.150 "
        "threshold_m=1.000"
    )
