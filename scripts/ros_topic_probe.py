#!/usr/bin/env python3
"""Wait for ROS 2 topics, print their types, and confirm one message each."""

from __future__ import annotations

import argparse
import sys
import time


PASS_LINE = "PASS: DFD-010 ROS visualization state topics passed"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Wait for expected ROS 2 topics, subscribe using their discovered "
            "message types, and print the type received for each topic."
        )
    )
    parser.add_argument("topics", nargs="+", help="Topic names to wait for.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for all topics and one message from each.",
    )
    return parser.parse_args()


def topic_type_map(node: object) -> dict[str, tuple[str, ...]]:
    return {
        topic_name: tuple(topic_types)
        for topic_name, topic_types in node.get_topic_names_and_types()
    }


def main() -> int:
    args = parse_args()
    if args.timeout <= 0.0:
        print("--timeout must be positive", file=sys.stderr)
        return 2

    try:
        import rclpy
        from rclpy.node import Node
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        print(f"ROS 2 import failed: {exc}", file=sys.stderr)
        return 2

    class TopicProbe(Node):
        def __init__(self, topics: list[str]) -> None:
            super().__init__("ros_topic_probe")
            self.expected_topics = tuple(topics)
            self.discovered_types: dict[str, tuple[str, ...]] = {}
            self.received_types: dict[str, str] = {}
            self._subscriptions_by_topic: dict[str, object] = {}

        def refresh_subscriptions(self) -> None:
            topics = topic_type_map(self)
            for topic in self.expected_topics:
                topic_types = topics.get(topic)
                if not topic_types:
                    continue
                self.discovered_types[topic] = topic_types
                if topic in self._subscriptions_by_topic:
                    continue
                message_type_name = topic_types[0]
                message_type = get_message(message_type_name)
                self._subscriptions_by_topic[topic] = self.create_subscription(
                    message_type,
                    topic,
                    self._callback_for(topic, message_type_name),
                    10,
                )

        def _callback_for(self, topic: str, message_type_name: str):
            def on_message(_message: object) -> None:
                self.received_types[topic] = message_type_name

            return on_message

        @property
        def missing_topics(self) -> tuple[str, ...]:
            return tuple(
                topic
                for topic in self.expected_topics
                if topic not in self.discovered_types
            )

        @property
        def missing_messages(self) -> tuple[str, ...]:
            return tuple(
                topic
                for topic in self.expected_topics
                if topic not in self.received_types
            )

        @property
        def complete(self) -> bool:
            return not self.missing_topics and not self.missing_messages

    rclpy.init()
    node = TopicProbe(args.topics)
    deadline_s = time.monotonic() + args.timeout
    print(
        "Waiting for ROS topics and messages: "
        + ", ".join(node.expected_topics)
    )

    try:
        while rclpy.ok() and time.monotonic() < deadline_s:
            node.refresh_subscriptions()
            if node.complete:
                break
            rclpy.spin_once(node, timeout_sec=0.1)

        node.refresh_subscriptions()
        if not node.complete:
            if node.missing_topics:
                print(
                    "Missing topics: " + ", ".join(node.missing_topics),
                    file=sys.stderr,
                )
            if node.missing_messages:
                print(
                    "Missing messages: " + ", ".join(node.missing_messages),
                    file=sys.stderr,
                )
            return 1

        for topic in node.expected_topics:
            print(f"{topic} type={node.received_types[topic]} received=yes")
        print(PASS_LINE)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
