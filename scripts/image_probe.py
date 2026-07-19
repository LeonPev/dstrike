#!/usr/bin/env python3
"""Probe a ROS 2 Image topic and print frame statistics."""

from __future__ import annotations

import argparse
import sys
import time


PASS_LINE = "PASS: DFD-011 MuJoCo camera publishing passed"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Subscribe to a ROS 2 sensor_msgs/Image topic and print width, "
            "height, encoding, FPS estimate, and pixel min/max."
        )
    )
    parser.add_argument("topic", help="Image topic to probe.")
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Number of image samples to collect.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for image samples.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples < 1:
        print("--samples must be at least 1", file=sys.stderr)
        return 2
    if args.timeout <= 0.0:
        print("--timeout must be positive", file=sys.stderr)
        return 2

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except ImportError as exc:
        print(f"ROS 2 import failed: {exc}", file=sys.stderr)
        return 2

    class ImageProbe(Node):
        def __init__(self, topic: str) -> None:
            super().__init__("image_probe")
            self.messages: list[Image] = []
            self.received_at_s: list[float] = []
            self.create_subscription(Image, topic, self.on_image, 10)

        def on_image(self, message: Image) -> None:
            self.messages.append(message)
            self.received_at_s.append(time.monotonic())

    rclpy.init()
    node = ImageProbe(args.topic)
    deadline_s = time.monotonic() + args.timeout
    print(f"Waiting for {args.samples} image samples from {args.topic}")

    try:
        while (
            rclpy.ok()
            and len(node.messages) < args.samples
            and time.monotonic() < deadline_s
        ):
            rclpy.spin_once(node, timeout_sec=0.1)

        if len(node.messages) < args.samples:
            print(
                f"Timed out after {args.timeout:g}s with "
                f"{len(node.messages)}/{args.samples} image samples",
                file=sys.stderr,
            )
            return 1

        latest = node.messages[-1]
        pixel_bytes = bytes(latest.data)
        expected_min_size = int(latest.height) * int(latest.step)
        if latest.width <= 0 or latest.height <= 0:
            print("Image width and height must be positive", file=sys.stderr)
            return 1
        if not pixel_bytes:
            print("Image data buffer is empty", file=sys.stderr)
            return 1
        if len(pixel_bytes) < expected_min_size:
            print(
                "Image data buffer is smaller than height * step: "
                f"len={len(pixel_bytes)} expected_at_least={expected_min_size}",
                file=sys.stderr,
            )
            return 1

        elapsed_s = node.received_at_s[-1] - node.received_at_s[0]
        fps_estimate = (
            (len(node.received_at_s) - 1) / elapsed_s
            if len(node.received_at_s) > 1 and elapsed_s > 0.0
            else 0.0
        )
        pixel_min = min(pixel_bytes)
        pixel_max = max(pixel_bytes)
        pixel_mean = sum(pixel_bytes) / len(pixel_bytes)
        dark_fraction = sum(value <= 10 for value in pixel_bytes) / len(pixel_bytes)
        print(
            f"topic={args.topic} "
            f"samples={len(node.messages)} "
            f"width={latest.width} "
            f"height={latest.height} "
            f"encoding={latest.encoding} "
            f"step={latest.step} "
            f"fps_estimate={fps_estimate:.2f} "
            f"pixel_min={pixel_min} "
            f"pixel_max={pixel_max} "
            f"pixel_mean={pixel_mean:.2f} "
            f"dark_fraction={dark_fraction:.3f}"
        )
        print(PASS_LINE)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
