#!/usr/bin/env python3
"""List ROS 2 topics and assert the DFD-015 observability topic set."""

from __future__ import annotations

import argparse
import sys
import time


PASS_LINE = "PASS: DFD-015 topic list passed"


def topic_type_map(node: object) -> dict[str, tuple[str, ...]]:
    return {
        topic_name: tuple(topic_types)
        for topic_name, topic_types in node.get_topic_names_and_types()
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Print the ROS 2 topic list with types, require the DFD-015 "
            "debugging topics, and note optional topics when present."
        )
    )
    parser.add_argument(
        "--required",
        nargs="+",
        default=[
            "/drone/odom",
            "/sim/ground_truth/odom",
            "/tf",
            "/sim/scenario_markers",
        ],
        help="Required topic names that must be visible.",
    )
    parser.add_argument(
        "--optional",
        nargs="*",
        default=["/sim/camera/front/image_raw"],
        help="Optional topic names to report when visible.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for required topics to appear.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0.0:
        print("--timeout must be positive", file=sys.stderr)
        return 2

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        print(f"ROS 2 import failed: {exc}", file=sys.stderr)
        return 2

    class TopicListProbe(Node):
        def __init__(self) -> None:
            super().__init__("ros_topic_list_probe")

    rclpy.init()
    node = TopicListProbe()
    deadline_s = time.monotonic() + args.timeout
    topics: dict[str, tuple[str, ...]] = {}

    try:
        while rclpy.ok() and time.monotonic() < deadline_s:
            topics = topic_type_map(node)
            if all(topic in topics for topic in args.required):
                break
            rclpy.spin_once(node, timeout_sec=0.25)

        topics = topic_type_map(node)
        for topic_name in sorted(topics):
            print(f"{topic_name} [{', '.join(topics[topic_name])}]")

        missing = [topic for topic in args.required if topic not in topics]
        if missing:
            print("Missing required topics: " + ", ".join(missing), file=sys.stderr)
            return 1

        for topic in args.required:
            print(f"required-topic-visible={topic}")

        for topic in args.optional:
            if topic in topics:
                print(f"optional-topic-visible={topic}")
            else:
                print(f"optional-topic-missing={topic}")

        print(PASS_LINE)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
