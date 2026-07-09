#!/usr/bin/env python3
"""Run the DFD-014 end-to-end takeoff smoke test through ROS 2 topics."""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Callable


PASS_LINE = "E2E PASS: heartbeat, arm, takeoff, odom altitude threshold reached"


def parse_heartbeat_payload(payload: str) -> dict[str, Any] | None:
    """Return a bridge heartbeat dictionary when the payload looks valid."""

    try:
        heartbeat = json.loads(payload)
    except json.JSONDecodeError:
        return None

    if not isinstance(heartbeat, dict):
        return None
    if "sysid" not in heartbeat or "compid" not in heartbeat:
        return None
    return heartbeat


def altitude_threshold_reached(
    latest_altitude_m: float | None,
    threshold_m: float,
) -> bool:
    if latest_altitude_m is None:
        return False
    return float(latest_altitude_m) > float(threshold_m)


def altitude_summary(
    starting_altitude_m: float | None,
    latest_altitude_m: float | None,
    threshold_m: float,
) -> str:
    if starting_altitude_m is None:
        starting = "unknown"
    else:
        starting = f"{starting_altitude_m:.3f}"

    if latest_altitude_m is None:
        latest = "unknown"
        delta = "unknown"
    else:
        latest = f"{latest_altitude_m:.3f}"
        if starting_altitude_m is None:
            delta = "unknown"
        else:
            delta = f"{latest_altitude_m - starting_altitude_m:.3f}"

    return (
        f"starting_altitude_m={starting} "
        f"latest_altitude_m={latest} "
        f"delta_m={delta} "
        f"threshold_m={threshold_m:.3f}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Wait for the MAVLink heartbeat bridged into ROS 2, verify bridge "
            "topics, arm, command a 2 m takeoff, and pass when /drone/odom "
            "altitude exceeds the configured threshold."
        )
    )
    parser.add_argument(
        "--heartbeat-topic",
        default="/drone/heartbeat",
        help="Bridge heartbeat topic carrying MAVLink heartbeat JSON.",
    )
    parser.add_argument(
        "--odom-topic",
        default="/drone/odom",
        help="ROS ENU/z-up odometry topic to monitor.",
    )
    parser.add_argument(
        "--arm-topic",
        default="/drone/arm",
        help="Bool topic used to arm and disarm.",
    )
    parser.add_argument(
        "--takeoff-topic",
        default="/drone/takeoff_alt_m",
        help="Float32 topic used to command takeoff altitude.",
    )
    parser.add_argument(
        "--land-topic",
        default="/drone/land",
        help="Bool topic used for cleanup landing.",
    )
    parser.add_argument(
        "--takeoff-altitude",
        type=float,
        default=2.0,
        help="Commanded takeoff altitude in meters.",
    )
    parser.add_argument(
        "--min-altitude",
        type=float,
        default=1.0,
        help="Pass threshold for /drone/odom pose.pose.position.z in meters.",
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=180.0,
        help="Seconds to wait for the bridged MAVLink heartbeat.",
    )
    parser.add_argument(
        "--topics-timeout",
        type=float,
        default=120.0,
        help="Seconds to wait for bridge publishers/subscriptions.",
    )
    parser.add_argument(
        "--arm-timeout",
        type=float,
        default=90.0,
        help="Seconds to wait for heartbeat armed=true after publishing arm.",
    )
    parser.add_argument(
        "--takeoff-timeout",
        type=float,
        default=90.0,
        help="Seconds to wait for odometry altitude to exceed the threshold.",
    )
    parser.add_argument(
        "--command-duration",
        type=float,
        default=1.5,
        help="Seconds to repeat each command publication for DDS discovery.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> bool:
    positive_float_fields = (
        "takeoff_altitude",
        "min_altitude",
        "heartbeat_timeout",
        "topics_timeout",
        "arm_timeout",
        "takeoff_timeout",
        "command_duration",
    )
    for field in positive_float_fields:
        if getattr(args, field) <= 0.0:
            print(f"--{field.replace('_', '-')} must be positive", file=sys.stderr)
            return False
    return True


def main() -> int:
    args = parse_args()
    if not validate_args(args):
        return 2

    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from std_msgs.msg import Bool, Float32, String
    except ImportError as exc:
        print(f"ROS 2 import failed: {exc}", file=sys.stderr)
        return 2

    class E2ETakeoffNode(Node):
        def __init__(self) -> None:
            super().__init__("e2e_takeoff_smoke")
            self.last_heartbeat: dict[str, Any] | None = None
            self.heartbeat_count = 0
            self.starting_altitude_m: float | None = None
            self.latest_altitude_m: float | None = None
            self.odom_sample_count = 0

            self.create_subscription(
                String,
                args.heartbeat_topic,
                self._on_heartbeat,
                10,
            )
            self.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)
            self.arm_pub = self.create_publisher(Bool, args.arm_topic, 10)
            self.takeoff_pub = self.create_publisher(
                Float32,
                args.takeoff_topic,
                10,
            )
            self.land_pub = self.create_publisher(Bool, args.land_topic, 10)

        def _on_heartbeat(self, message: String) -> None:
            heartbeat = parse_heartbeat_payload(message.data)
            if heartbeat is None:
                return
            self.last_heartbeat = heartbeat
            self.heartbeat_count += 1

        def _on_odom(self, message: Odometry) -> None:
            altitude_m = float(message.pose.pose.position.z)
            if self.starting_altitude_m is None:
                self.starting_altitude_m = altitude_m
            self.latest_altitude_m = altitude_m
            self.odom_sample_count += 1

        @property
        def armed(self) -> bool:
            if self.last_heartbeat is None:
                return False
            return bool(self.last_heartbeat.get("armed"))

        def missing_bridge_endpoints(self) -> list[str]:
            missing: list[str] = []
            for topic in (args.heartbeat_topic, args.odom_topic):
                if self.count_publishers(topic) < 1:
                    missing.append(f"publisher:{topic}")

            for topic in (args.arm_topic, args.takeoff_topic, args.land_topic):
                if self.count_subscribers(topic) < 1:
                    missing.append(f"subscriber:{topic}")

            return missing

    def spin_until(
        node: E2ETakeoffNode,
        timeout_s: float,
        predicate: Callable[[], bool],
    ) -> bool:
        deadline_s = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline_s:
            if predicate():
                return True
            remaining_s = deadline_s - time.monotonic()
            rclpy.spin_once(node, timeout_sec=max(0.01, min(0.1, remaining_s)))
        return predicate()

    def publish_for(
        node: E2ETakeoffNode,
        publisher: Any,
        message: Any,
        duration_s: float,
    ) -> None:
        deadline_s = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline_s:
            publisher.publish(message)
            remaining_s = deadline_s - time.monotonic()
            rclpy.spin_once(node, timeout_sec=max(0.01, min(0.1, remaining_s)))

    def cleanup(node: E2ETakeoffNode) -> None:
        print("Sending cleanup land/disarm commands")
        publish_for(node, node.land_pub, Bool(data=True), args.command_duration)
        publish_for(node, node.arm_pub, Bool(data=False), args.command_duration)
        print("cleanup commands published")

    rclpy.init()
    node = E2ETakeoffNode()
    cleanup_done = False
    try:
        print(
            f"Waiting for MAVLink heartbeat via {args.heartbeat_topic} "
            f"within {args.heartbeat_timeout:g}s"
        )
        if not spin_until(
            node,
            args.heartbeat_timeout,
            lambda: node.last_heartbeat is not None,
        ):
            print(
                f"Timed out waiting for MAVLink heartbeat on {args.heartbeat_topic}",
                file=sys.stderr,
            )
            return 1

        heartbeat = node.last_heartbeat or {}
        print(
            "heartbeat "
            f"sysid={heartbeat.get('sysid')} "
            f"compid={heartbeat.get('compid')} "
            f"mode={heartbeat.get('mode', 'UNKNOWN')} "
            f"armed={heartbeat.get('armed')}"
        )

        print("Waiting for ROS 2 bridge publishers/subscriptions")
        if not spin_until(
            node,
            args.topics_timeout,
            lambda: not node.missing_bridge_endpoints(),
        ):
            print(
                "Missing bridge endpoints: "
                + ", ".join(node.missing_bridge_endpoints()),
                file=sys.stderr,
            )
            return 1
        print("bridge endpoints ready")

        print(f"Waiting for initial odometry on {args.odom_topic}")
        if not spin_until(
            node,
            args.topics_timeout,
            lambda: node.latest_altitude_m is not None,
        ):
            print(
                f"Timed out waiting for odometry on {args.odom_topic}",
                file=sys.stderr,
            )
            return 1
        print(
            altitude_summary(
                node.starting_altitude_m,
                node.latest_altitude_m,
                args.min_altitude,
            )
        )

        print(f"Publishing arm command on {args.arm_topic}")
        publish_for(node, node.arm_pub, Bool(data=True), args.command_duration)
        if not spin_until(node, args.arm_timeout, lambda: node.armed):
            print(
                f"Timed out waiting for armed=true in {args.heartbeat_topic}",
                file=sys.stderr,
            )
            return 1
        print("armed=true")

        print(
            f"Publishing takeoff command altitude_m={args.takeoff_altitude:g} "
            f"on {args.takeoff_topic}"
        )
        odom_samples_before_takeoff = node.odom_sample_count
        publish_for(
            node,
            node.takeoff_pub,
            Float32(data=float(args.takeoff_altitude)),
            args.command_duration,
        )

        def takeoff_reached() -> bool:
            if node.odom_sample_count <= odom_samples_before_takeoff:
                return False
            return altitude_threshold_reached(
                node.latest_altitude_m,
                args.min_altitude,
            )

        if not spin_until(node, args.takeoff_timeout, takeoff_reached):
            print(
                "Timed out waiting for odometry altitude threshold: "
                + altitude_summary(
                    node.starting_altitude_m,
                    node.latest_altitude_m,
                    args.min_altitude,
                ),
                file=sys.stderr,
            )
            return 1

        print(
            altitude_summary(
                node.starting_altitude_m,
                node.latest_altitude_m,
                args.min_altitude,
            )
        )
        cleanup(node)
        cleanup_done = True
        print(PASS_LINE)
        return 0
    finally:
        if not cleanup_done:
            try:
                cleanup(node)
            except Exception as exc:  # pragma: no cover - best-effort live cleanup.
                print(f"cleanup failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
