#!/usr/bin/env python3
"""Measure a ROS 2 topic publication rate over a bounded sample window."""

from __future__ import annotations

import argparse
import statistics
import sys
import time


PASS_LINE = "PASS: DFD-015 topic rate probe passed"


def calculate_rate_hz(sample_times_s: list[float]) -> float:
    if len(sample_times_s) < 2:
        return 0.0
    elapsed_s = sample_times_s[-1] - sample_times_s[0]
    if elapsed_s <= 0.0:
        return 0.0
    return (len(sample_times_s) - 1) / elapsed_s


def calculate_jitter_s(sample_times_s: list[float]) -> float:
    if len(sample_times_s) < 3:
        return 0.0
    intervals = [
        sample_times_s[index] - sample_times_s[index - 1]
        for index in range(1, len(sample_times_s))
    ]
    return statistics.pstdev(intervals)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Discover a topic type, subscribe to the topic, and report a bounded "
            "publication-rate estimate suitable for Makefile smoke checks."
        )
    )
    parser.add_argument("topic", help="ROS 2 topic to sample.")
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Seconds to sample after the first received message.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Maximum seconds to wait for discovery and samples.",
    )
    parser.add_argument(
        "--min-rate-hz",
        type=float,
        default=0.1,
        help="Minimum average publication rate required to pass.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> bool:
    if args.duration <= 0.0:
        print("--duration must be positive", file=sys.stderr)
        return False
    if args.timeout <= args.duration:
        print("--timeout must be greater than --duration", file=sys.stderr)
        return False
    if args.min_rate_hz <= 0.0:
        print("--min-rate-hz must be positive", file=sys.stderr)
        return False
    return True


def main() -> int:
    args = parse_args()
    if not validate_args(args):
        return 2

    try:
        import rclpy
        from rclpy.node import Node
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        print(f"ROS 2 import failed: {exc}", file=sys.stderr)
        return 2

    class TopicHzProbe(Node):
        def __init__(self) -> None:
            super().__init__("ros_topic_hz_probe")
            self.message_type_name: str | None = None
            self.sample_times_s: list[float] = []
            self._subscription: object | None = None

        def refresh_subscription(self) -> None:
            if self._subscription is not None:
                return

            topic_types = dict(self.get_topic_names_and_types()).get(args.topic)
            if not topic_types:
                return

            self.message_type_name = topic_types[0]
            message_type = get_message(self.message_type_name)
            self._subscription = self.create_subscription(
                message_type,
                args.topic,
                self._on_message,
                10,
            )

        def _on_message(self, _message: object) -> None:
            self.sample_times_s.append(time.monotonic())

    rclpy.init()
    node = TopicHzProbe()
    deadline_s = time.monotonic() + args.timeout
    sample_deadline_s: float | None = None
    print(
        f"Sampling topic={args.topic} duration_s={args.duration:g} "
        f"timeout_s={args.timeout:g}"
    )

    try:
        while rclpy.ok() and time.monotonic() < deadline_s:
            node.refresh_subscription()
            if node.sample_times_s and sample_deadline_s is None:
                sample_deadline_s = time.monotonic() + args.duration
            if sample_deadline_s is not None and time.monotonic() >= sample_deadline_s:
                break
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.message_type_name is None:
            print(f"Topic not discovered: {args.topic}", file=sys.stderr)
            return 1
        if len(node.sample_times_s) < 2:
            print(
                f"Too few samples for {args.topic}: count={len(node.sample_times_s)}",
                file=sys.stderr,
            )
            return 1

        rate_hz = calculate_rate_hz(node.sample_times_s)
        jitter_s = calculate_jitter_s(node.sample_times_s)
        elapsed_s = node.sample_times_s[-1] - node.sample_times_s[0]
        print(
            f"{args.topic} type={node.message_type_name} "
            f"count={len(node.sample_times_s)} "
            f"elapsed_s={elapsed_s:.3f} "
            f"avg_rate_hz={rate_hz:.3f} "
            f"jitter_s={jitter_s:.4f}"
        )
        if rate_hz < args.min_rate_hz:
            print(
                f"Rate below minimum: avg_rate_hz={rate_hz:.3f} "
                f"min_rate_hz={args.min_rate_hz:.3f}",
                file=sys.stderr,
            )
            return 1

        print(PASS_LINE)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
