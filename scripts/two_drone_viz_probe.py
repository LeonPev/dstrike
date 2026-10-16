#!/usr/bin/env python3
"""Bounded SCN-007 ROS graph and noVNC readiness probe."""

import argparse
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node


REQUIRED_TOPIC_TYPES = {
    "/sim/target/odom": "nav_msgs/msg/Odometry",
    "/sim/chaser/ground_truth/odom": "nav_msgs/msg/Odometry",
    "/drone/odom": "nav_msgs/msg/Odometry",
    "/sim/scenario_markers": "visualization_msgs/msg/MarkerArray",
    "/sim/drone_markers": "visualization_msgs/msg/MarkerArray",
    "/sim/target/camera/front/image_raw": "sensor_msgs/msg/Image",
    "/sim/chaser/camera/front/image_raw": "sensor_msgs/msg/Image",
    "/tf": "tf2_msgs/msg/TFMessage",
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--novnc-url", default="http://viz:6080/vnc.html")
    return parser.parse_args()


def novnc_ready(url):
    try:
        with urllib.request.urlopen(url, timeout=2.0) as response:
            return response.status == 200 and b"noVNC" in response.read(65536)
    except (OSError, urllib.error.URLError):
        return False


def main():
    args = parse_args()
    rclpy.init()
    node = Node("scn007_two_drone_viz_probe")
    deadline = time.monotonic() + args.timeout_s
    missing = dict(REQUIRED_TOPIC_TYPES)
    web_ready = False
    try:
        while time.monotonic() < deadline:
            graph = dict(node.get_topic_names_and_types())
            missing = {
                topic: expected
                for topic, expected in REQUIRED_TOPIC_TYPES.items()
                if expected not in graph.get(topic, [])
            }
            web_ready = novnc_ready(args.novnc_url)
            if not missing and web_ready:
                print("SCN-007 probe: required ROS topics and noVNC are ready")
                return 0
            rclpy.spin_once(node, timeout_sec=0.25)
        if missing:
            print("ERROR: missing or mistyped required topics:")
            for topic, expected in sorted(missing.items()):
                print(f"  {topic}: expected {expected}")
        if not web_ready:
            print(f"ERROR: noVNC did not become ready at {args.novnc_url}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
