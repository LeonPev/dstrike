#!/usr/bin/env python3
"""Bounded ROS probe for distinct SCN-010 vehicle identities and odometry."""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


PASS_LINE = "PASS: SCN-010 optional dual-ArduPilot SITL setup works with separate vehicle identities"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=120.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node("dual_sitl_probe")
    heartbeats: dict[str, dict] = {}
    odometry: set[str] = set()

    def heartbeat(role):
        def receive(message):
            try:
                heartbeats[role] = json.loads(message.data)
            except json.JSONDecodeError:
                pass
        return receive

    subscriptions = []
    for role in ("target", "chaser"):
        subscriptions.append(node.create_subscription(String, f"/{role}/drone/heartbeat", heartbeat(role), 10))
        subscriptions.append(node.create_subscription(Odometry, f"/{role}/drone/odom", lambda _message, role=role: odometry.add(role), 10))

    deadline = time.monotonic() + args.timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(heartbeats) == 2 and len(odometry) == 2:
                break
        missing_heartbeat = {"target", "chaser"} - heartbeats.keys()
        missing_odom = {"target", "chaser"} - odometry
        if missing_heartbeat or missing_odom:
            raise RuntimeError(f"missing heartbeats={sorted(missing_heartbeat)} odometry={sorted(missing_odom)}")
        sysids = {role: int(value.get("sysid", 0)) for role, value in heartbeats.items()}
        if sysids != {"target": 2, "chaser": 1}:
            raise RuntimeError(f"unexpected vehicle identities: {sysids}")
        print(f"heartbeats target_sysid={sysids['target']} chaser_sysid={sysids['chaser']}")
        print("odometry target=true chaser=true")
        print(PASS_LINE)
        return 0
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
