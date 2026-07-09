"""ArduPilot JSON external-physics protocol stub.

This service intentionally does not simulate motion yet. It only speaks the
UDP protocol shape ArduPilot expects so DFD-005 can prove the SITL JSON path.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import socket
import struct
import sys
import time
from typing import TextIO


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 9002
LOG_INTERVAL_SECONDS = 1.0
STARTUP_LOG_PACKETS = 3

MAGIC_PWM_16 = 18458
MAGIC_PWM_32 = 29569

PACKET_16 = struct.Struct("<HHI16H")
PACKET_32 = struct.Struct("<HHI32H")

class PacketError(ValueError):
    """Raised when an incoming UDP payload is not an ArduPilot PWM packet."""


@dataclass(frozen=True)
class PwmPacket:
    magic: int
    frame_rate: int
    frame_count: int
    pwm: tuple[int, ...]

    @property
    def is_32_channel(self) -> bool:
        return self.magic == MAGIC_PWM_32


def _exact_payload(payload: bytes, expected_size: int, packet_name: str) -> bytes:
    if len(payload) != expected_size:
        raise PacketError(
            f"{packet_name} packet has {len(payload)} bytes; expected {expected_size}"
        )
    return payload


def parse_pwm_packet(payload: bytes) -> PwmPacket:
    """Parse one ArduPilot servo-output packet."""
    if len(payload) < 2:
        raise PacketError(f"packet too short: {len(payload)} bytes")

    (magic,) = struct.unpack_from("<H", payload)
    if magic == MAGIC_PWM_16:
        fields = PACKET_16.unpack(_exact_payload(payload, PACKET_16.size, "16-channel"))
    elif magic == MAGIC_PWM_32:
        fields = PACKET_32.unpack(_exact_payload(payload, PACKET_32.size, "32-channel"))
    else:
        raise PacketError(f"unknown magic {magic}")

    parsed_magic, frame_rate, frame_count, *pwm = fields
    return PwmPacket(
        magic=parsed_magic,
        frame_rate=frame_rate,
        frame_count=frame_count,
        pwm=tuple(pwm),
    )


def static_physics_state(timestamp: float) -> dict[str, object]:
    """Return a valid static ArduPilot JSON physics state."""
    return {
        "timestamp": timestamp,
        "imu": {
            "gyro": [0.0, 0.0, 0.0],
            "accel_body": [0.0, 0.0, -9.80665],
        },
        "position": [0.0, 0.0, 0.0],
        "velocity": [0.0, 0.0, 0.0],
        "quaternion": [1.0, 0.0, 0.0, 0.0],
    }


def build_state_json(timestamp: float) -> bytes:
    payload = json.dumps(
        static_physics_state(timestamp),
        separators=(",", ":"),
    )
    return f"{payload}\n".encode("utf-8")


def log_packet(packet: PwmPacket, sender: tuple[str, int], stream: TextIO) -> None:
    first4 = ",".join(str(value) for value in packet.pwm[:4])
    mode = "pwm32-detected-not-implemented" if packet.is_32_channel else "pwm16"
    print(
        "pwm_packet "
        f"sender={sender[0]}:{sender[1]} "
        f"mode={mode} "
        f"magic={packet.magic} "
        f"frame_rate={packet.frame_rate} "
        f"frame_count={packet.frame_count} "
        f"pwm_first4=[{first4}]",
        file=stream,
        flush=True,
    )


def should_log_packet(
    packet: PwmPacket,
    logged_packets: int,
    now: float,
    last_log_at: float,
) -> bool:
    if packet.is_32_channel:
        return True
    if logged_packets < STARTUP_LOG_PACKETS:
        return True
    return now - last_log_at >= LOG_INTERVAL_SECONDS


def serve(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
) -> None:
    started_at = time.monotonic()
    last_log_at = 0.0
    logged_packets = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind((host, port))
        print(
            f"ArduPilot JSON physics stub listening on udp://{host}:{port}",
            file=stdout,
            flush=True,
        )

        while True:
            payload, sender = udp.recvfrom(4096)
            try:
                packet = parse_pwm_packet(payload)
            except PacketError as exc:
                print(
                    f"invalid_pwm_packet sender={sender[0]}:{sender[1]} error={exc}",
                    file=stderr,
                    flush=True,
                )
                continue

            now = time.monotonic()
            if should_log_packet(packet, logged_packets, now, last_log_at):
                log_packet(packet, sender, stdout)
                last_log_at = now
                logged_packets += 1

            timestamp = now - started_at
            udp.sendto(build_state_json(timestamp), sender)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve a minimal ArduPilot JSON external-physics endpoint."
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="UDP bind host.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP bind port.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        serve(host=args.host, port=args.port)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
