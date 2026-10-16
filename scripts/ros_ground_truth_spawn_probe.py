#!/usr/bin/env python3
"""Subscriber-first verifier for startup and repeated reset spawn samples."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--heading", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("map004_ground_truth_spawn_probe")
    matches = 0
    last_match_ns: int | None = None

    def receive(message: Odometry) -> None:
        nonlocal matches, last_match_ns
        pose = message.pose.pose
        twist = message.twist.twist
        yaw = math.degrees(2.0 * math.atan2(pose.orientation.z, pose.orientation.w))
        position_ok = (
            abs(pose.position.x - args.x) <= 0.25
            and abs(pose.position.y - args.y) <= 0.25
            and abs(pose.position.z - args.z) <= 0.25
        )
        heading_ok = abs(((yaw - args.heading + 180.0) % 360.0) - 180.0) <= 1.0
        pre_step = (
            abs(twist.linear.x) < 1e-9
            and abs(twist.linear.y) < 1e-9
            and abs(twist.linear.z) < 1e-9
        )
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if position_ok and heading_ok and pre_step and stamp_ns != last_match_ns:
            matches += 1
            last_match_ns = stamp_ns
            labels = ("startup", "reset-1", "reset-2")
            print(f"SPAWN {labels[min(matches - 1, 2)]} x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f} heading={yaw:.3f}", flush=True)

    subscription = node.create_subscription(Odometry, "/sim/ground_truth/odom", receive, 10)
    print("READY: /sim/ground_truth/odom subscription active", flush=True)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and matches < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        del subscription
        node.destroy_node()
        rclpy.shutdown()
    if matches < 3:
        raise SystemExit(f"timed out after observing {matches}/3 pre-step spawn samples")
    print("PASS: live startup and two reset spawn samples match configured terrain clearance", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
