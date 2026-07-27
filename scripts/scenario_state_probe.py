#!/usr/bin/env python3
"""Bounded SCN-008 state and late-subscriber probe."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=90.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node("scn008_state_probe")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    states = []
    received_at = []

    def receive(message):
        if not states or states[-1] != message.data:
            print(f"scenario-state={message.data}", flush=True)
            states.append(message.data)
            received_at.append(time.monotonic())

    discovery_deadline = time.monotonic() + 10.0
    while time.monotonic() < discovery_deadline and not node.get_publishers_info_by_topic("/scenario/state"):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.get_publishers_info_by_topic("/scenario/state"):
        print("ERROR: scenario state publisher was not discovered", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    subscribed_at = time.monotonic()
    subscription = node.create_subscription(String, "/scenario/state", receive, qos)
    deadline = subscribed_at + args.timeout_s
    try:
        while time.monotonic() < deadline and "INTERCEPT" not in states and "ABORT" not in states:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not states or received_at[0] - subscribed_at > 2.0:
            raise RuntimeError("late subscriber did not receive current state within 2 seconds")
        if "ABORT" in states:
            raise RuntimeError("controller entered ABORT")
        if "INTERCEPT" not in states:
            raise RuntimeError(f"controller did not reach INTERCEPT; observed={states}")
        print(f"observed-states={','.join(states)}")
        print("late-subscriber-state-observed=true")
        return 0
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
