from __future__ import annotations

import json

from src.physics.ardupilot_json_stub import (
    MAGIC_PWM_16,
    MAGIC_PWM_32,
    PACKET_16,
    PACKET_32,
    build_state_json,
    parse_pwm_packet,
)


def test_parse_16_channel_pwm_packet() -> None:
    payload = PACKET_16.pack(MAGIC_PWM_16, 400, 123, *range(1000, 1016))

    packet = parse_pwm_packet(payload)

    assert packet.magic == MAGIC_PWM_16
    assert packet.frame_rate == 400
    assert packet.frame_count == 123
    assert packet.pwm[:4] == (1000, 1001, 1002, 1003)
    assert len(packet.pwm) == 16
    assert not packet.is_32_channel


def test_parse_32_channel_magic_is_detected() -> None:
    payload = PACKET_32.pack(MAGIC_PWM_32, 400, 124, *range(1100, 1132))

    packet = parse_pwm_packet(payload)

    assert packet.magic == MAGIC_PWM_32
    assert packet.frame_rate == 400
    assert packet.frame_count == 124
    assert packet.pwm[:4] == (1100, 1101, 1102, 1103)
    assert len(packet.pwm) == 32
    assert packet.is_32_channel


def test_state_json_is_newline_terminated_and_contains_mandatory_fields() -> None:
    payload = build_state_json(1.25)

    assert payload.endswith(b"\n")
    state = json.loads(payload)
    assert state["timestamp"] == 1.25
    assert state["imu"]["gyro"] == [0.0, 0.0, 0.0]
    assert state["imu"]["accel_body"] == [0.0, 0.0, -9.80665]
    assert state["position"] == [0.0, 0.0, 0.0]
    assert state["velocity"] == [0.0, 0.0, 0.0]
    assert state["quaternion"] == [1.0, 0.0, 0.0, 0.0]
