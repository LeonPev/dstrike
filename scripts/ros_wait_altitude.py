#!/usr/bin/env python3
"""Wait for ROS ENU/z-up odometry altitude to increase by a threshold."""

from __future__ import annotations

import argparse
import sys
import time


PASS_LINE = "PASS: DFD-009 ROS arm and takeoff passed"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Subscribe to nav_msgs/Odometry and wait for pose.pose.position.z "
            "to increase by the requested ENU z-up delta."
        )
    )
    parser.add_argument(
        "--topic",
        default="/drone/odom",
        help="Odometry topic to watch.",
    )
    parser.add_argument(
        "--min-delta",
        type=float,
        default=0.5,
        help="Minimum z-up altitude increase in meters.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for the altitude increase.",
    )
    return parser.parse_args()


def altitude_delta(starting_altitude_m: float, latest_altitude_m: float) -> float:
    return float(latest_altitude_m) - float(starting_altitude_m)


def altitude_summary(starting_altitude_m: float, latest_altitude_m: float) -> str:
    delta_m = altitude_delta(starting_altitude_m, latest_altitude_m)
    return (
        f"starting_altitude_m={starting_altitude_m:.3f} "
        f"latest_altitude_m={latest_altitude_m:.3f} "
        f"delta_m={delta_m:.3f}"
    )


def main() -> int:
    args = parse_args()
    if args.min_delta <= 0.0:
        print("--min-delta must be positive", file=sys.stderr)
        return 2
    if args.timeout <= 0.0:
        print("--timeout must be positive", file=sys.stderr)
        return 2

    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
    except ImportError as exc:
        print(f"ROS 2 import failed: {exc}", file=sys.stderr)
        return 2

    class AltitudeWatcher(Node):
        def __init__(self, topic: str) -> None:
            super().__init__("ros_wait_altitude")
            self.starting_altitude_m: float | None = None
            self.latest_altitude_m: float | None = None
            self.create_subscription(Odometry, topic, self._on_odom, 10)

        def _on_odom(self, message: Odometry) -> None:
            altitude_m = float(message.pose.pose.position.z)
            if self.starting_altitude_m is None:
                self.starting_altitude_m = altitude_m
            self.latest_altitude_m = altitude_m

        @property
        def delta_m(self) -> float | None:
            if self.starting_altitude_m is None or self.latest_altitude_m is None:
                return None
            return altitude_delta(self.starting_altitude_m, self.latest_altitude_m)

    rclpy.init()
    node = AltitudeWatcher(args.topic)
    deadline_s = time.monotonic() + args.timeout
    print(
        f"Waiting for {args.topic} altitude delta >= {args.min_delta:g}m "
        f"within {args.timeout:g}s"
    )

    try:
        while rclpy.ok():
            remaining_s = deadline_s - time.monotonic()
            if remaining_s <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining_s))
            if node.delta_m is not None and node.delta_m >= args.min_delta:
                break

        if node.starting_altitude_m is None or node.latest_altitude_m is None:
            print(
                f"Timed out after {args.timeout:g}s waiting for {args.topic}",
                file=sys.stderr,
            )
            return 1

        print(altitude_summary(node.starting_altitude_m, node.latest_altitude_m))
        if node.delta_m is not None and node.delta_m >= args.min_delta:
            print(PASS_LINE)
            return 0

        print(
            f"Timed out after {args.timeout:g}s before altitude delta reached "
            f"{args.min_delta:g}m",
            file=sys.stderr,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
