#!/usr/bin/env python3
"""Wait for one MAVLink heartbeat and print the fields needed by DFD-004."""

from __future__ import annotations

import argparse
import sys
import time


PASS_LINE = "PASS: DFD-004 baseline SITL heartbeat passed"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Wait for a MAVLink HEARTBEAT on a UDP listener."
    )
    parser.add_argument(
        "--connect",
        default="udp:0.0.0.0:14550",
        help="MAVLink connection string. udp:HOST:PORT is treated as a listener.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for the heartbeat.",
    )
    return parser.parse_args()


def listener_endpoint(connect: str) -> str:
    if connect.startswith("udp:"):
        return "udpin:" + connect[len("udp:") :]
    return connect


def enum_label(mavutil, enum_name: str, value: int) -> str:
    enum = mavutil.mavlink.enums.get(enum_name, {})
    entry = enum.get(value)
    if entry is None:
        return str(value)
    return f"{value} ({entry.name})"


def wait_for_heartbeat(connection, timeout: float):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        heartbeat = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=min(1.0, remaining),
        )
        if heartbeat is not None:
            return heartbeat


def main() -> int:
    args = parse_args()

    try:
        from pymavlink import mavutil
    except ImportError as exc:
        print(f"pymavlink import failed: {exc}", file=sys.stderr)
        return 2

    endpoint = listener_endpoint(args.connect)
    print(f"Waiting for MAVLink heartbeat on {args.connect} ({endpoint})")

    connection = mavutil.mavlink_connection(endpoint, dialect="ardupilotmega")
    try:
        heartbeat = wait_for_heartbeat(connection, args.timeout)
    finally:
        connection.close()

    if heartbeat is None:
        print(
            f"Timed out after {args.timeout:g}s waiting for MAVLink HEARTBEAT",
            file=sys.stderr,
        )
        return 1

    print(
        "heartbeat "
        f"sysid={heartbeat.get_srcSystem()} "
        f"compid={heartbeat.get_srcComponent()} "
        f"vehicle_type={enum_label(mavutil, 'MAV_TYPE', heartbeat.type)} "
        f"autopilot_type={enum_label(mavutil, 'MAV_AUTOPILOT', heartbeat.autopilot)} "
        f"base_mode={heartbeat.base_mode} "
        f"custom_mode={heartbeat.custom_mode}"
    )
    print(PASS_LINE)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
