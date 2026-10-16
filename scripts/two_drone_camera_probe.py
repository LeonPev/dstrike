#!/usr/bin/env python3
"""Bounded live probe for both SCN-006 MuJoCo camera streams."""

from __future__ import annotations

import argparse
from collections import defaultdict
import math
import time
from typing import Any

import rclpy
from sensor_msgs.msg import CameraInfo, Image


IMAGE_TOPICS = {
    "/sim/target/camera/front/image_raw": "target/front_camera",
    "/sim/chaser/camera/front/image_raw": "chaser/front_camera",
}
INFO_TOPICS = {
    "/sim/target/camera/front/camera_info": "target/front_camera",
    "/sim/chaser/camera/front/camera_info": "chaser/front_camera",
}
MIN_PAIRED_FRAMES = 3


def fail(message: str) -> None:
    raise RuntimeError(message)


def validate_image(topic: str, expected_frame: str, message: Image) -> bytes:
    if message.header.frame_id != expected_frame:
        fail(f"{topic} frame_id={message.header.frame_id!r}, expected {expected_frame!r}")
    if message.encoding != "rgb8" or message.is_bigendian != 0:
        fail(f"{topic} must be little-endian rgb8")
    if message.width <= 0 or message.height <= 0 or message.step != message.width * 3:
        fail(f"{topic} has invalid dimensions or row step")
    data = bytes(message.data)
    if len(data) != message.height * message.step:
        fail(f"{topic} payload size does not match dimensions")
    if not data or min(data) == max(data):
        fail(f"{topic} frame is blank")
    return data


def validate_info(topic: str, expected_frame: str, message: CameraInfo) -> None:
    if message.header.frame_id != expected_frame:
        fail(f"{topic} frame_id={message.header.frame_id!r}, expected {expected_frame!r}")
    if message.width <= 0 or message.height <= 0:
        fail(f"{topic} has invalid dimensions")
    if message.distortion_model != "plumb_bob" or len(message.d) < 4:
        fail(f"{topic} has invalid distortion metadata")
    if len(message.k) != 9 or len(message.r) != 9 or len(message.p) != 12:
        fail(f"{topic} has invalid calibration matrix sizes")
    if not all(math.isfinite(value) for value in (*message.d, *message.k, *message.r, *message.p)):
        fail(f"{topic} calibration contains non-finite values")
    if message.k[0] <= 0.0 or message.k[4] <= 0.0 or message.k[8] != 1.0:
        fail(f"{topic} has invalid focal lengths or homogeneous scale")


def run(timeout_s: float) -> None:
    rclpy.init(args=None)
    node = rclpy.create_node("scn006_two_drone_camera_probe")
    frames: dict[str, list[bytes]] = defaultdict(list)
    infos: dict[str, CameraInfo] = {}
    subscriptions: list[Any] = []

    for topic, expected_frame in IMAGE_TOPICS.items():
        def on_image(message: Image, topic: str = topic, expected_frame: str = expected_frame) -> None:
            if len(frames[topic]) < MIN_PAIRED_FRAMES:
                frames[topic].append(validate_image(topic, expected_frame, message))

        subscriptions.append(node.create_subscription(Image, topic, on_image, 10))
    for topic, expected_frame in INFO_TOPICS.items():
        def on_info(message: CameraInfo, topic: str = topic, expected_frame: str = expected_frame) -> None:
            validate_info(topic, expected_frame, message)
            infos[topic] = message

        subscriptions.append(node.create_subscription(CameraInfo, topic, on_info, 10))

    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if all(len(frames[topic]) >= MIN_PAIRED_FRAMES for topic in IMAGE_TOPICS) and set(infos) == set(INFO_TOPICS):
                break
        else:
            counts = {topic: len(values) for topic, values in frames.items()}
            fail(f"timed out waiting for camera streams: frames={counts}, info={sorted(infos)}")

        target_frames = frames["/sim/target/camera/front/image_raw"]
        chaser_frames = frames["/sim/chaser/camera/front/image_raw"]
        for index, (target, chaser) in enumerate(zip(target_frames, chaser_frames), start=1):
            if target == chaser:
                fail(f"target and chaser images are byte-identical in paired frame {index}")
        print(
            f"camera_probe paired_frames={MIN_PAIRED_FRAMES} "
            f"target_bytes={len(target_frames[-1])} chaser_bytes={len(chaser_frames[-1])}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=60.0)
    args = parser.parse_args()
    try:
        run(args.timeout_s)
    except Exception as exc:
        print(f"ERROR: SCN-006 camera probe failed: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
