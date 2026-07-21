#!/usr/bin/env python3
"""Bounded live SCN-005 ROS topic, QoS, TF, alias, and marker probe."""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.physics.ros_telemetry import (  # noqa: E402
    CHASER_BASE_LINK_FRAME,
    CHASER_FRONT_CAMERA_FRAME,
    CHASER_GROUND_TRUTH_ODOM_TOPIC,
    DRONE_ODOM_TOPIC,
    GROUND_TRUTH_ODOM_TOPIC,
    TARGET_BASE_LINK_FRAME,
    TARGET_FRONT_CAMERA_FRAME,
    TARGET_ODOM_TOPIC,
    TF_TOPIC,
    GroundTruthOdomFields,
    odom_pose_errors,
)
from src.physics.ros_visualization import (  # noqa: E402
    DRONE_MARKERS_TOPIC,
    SCENARIO_MARKERS_TOPIC,
    WORLD_FRAME,
)


ODOM_TOPICS = (
    TARGET_ODOM_TOPIC,
    CHASER_GROUND_TRUTH_ODOM_TOPIC,
    GROUND_TRUTH_ODOM_TOPIC,
    DRONE_ODOM_TOPIC,
)
REQUIRED_TYPES = {
    TARGET_ODOM_TOPIC: "nav_msgs/msg/Odometry",
    CHASER_GROUND_TRUTH_ODOM_TOPIC: "nav_msgs/msg/Odometry",
    GROUND_TRUTH_ODOM_TOPIC: "nav_msgs/msg/Odometry",
    DRONE_ODOM_TOPIC: "nav_msgs/msg/Odometry",
    TF_TOPIC: "tf2_msgs/msg/TFMessage",
    SCENARIO_MARKERS_TOPIC: "visualization_msgs/msg/MarkerArray",
    DRONE_MARKERS_TOPIC: "visualization_msgs/msg/MarkerArray",
}
MARKER_TOPICS = (SCENARIO_MARKERS_TOPIC, DRONE_MARKERS_TOPIC)
MIN_ODOM_SAMPLES = 5
MIN_CORRELATION_BUFFER_SAMPLES = MIN_ODOM_SAMPLES * 2
MAX_POSITION_ERROR_M = 0.05
MAX_YAW_ERROR_DEG = 1.0
EXPECTED_ODOM_FRAMES = {
    TARGET_ODOM_TOPIC: (WORLD_FRAME, TARGET_BASE_LINK_FRAME),
    CHASER_GROUND_TRUTH_ODOM_TOPIC: (WORLD_FRAME, CHASER_BASE_LINK_FRAME),
    GROUND_TRUTH_ODOM_TOPIC: (WORLD_FRAME, "base_link"),
    DRONE_ODOM_TOPIC: (WORLD_FRAME, "base_link"),
}


@dataclass(frozen=True)
class TimedOdom:
    stamp_ns: int
    message: Odometry


@dataclass(frozen=True)
class PoseComparison:
    time_error_s: float
    position_error_m: float
    yaw_error_deg: float
    first: GroundTruthOdomFields
    second: GroundTruthOdomFields


def fail(message: str) -> None:
    raise RuntimeError(message)


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def odom_fields(message: Odometry) -> GroundTruthOdomFields:
    pose = message.pose.pose
    twist = message.twist.twist
    fields = GroundTruthOdomFields(
        position_enu=(pose.position.x, pose.position.y, pose.position.z),
        orientation_xyzw=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        linear_velocity_enu=(twist.linear.x, twist.linear.y, twist.linear.z),
    )
    values = (
        *fields.position_enu,
        *fields.orientation_xyzw,
        *fields.linear_velocity_enu,
    )
    if not all(math.isfinite(value) for value in values):
        fail("odometry pose/twist contains non-finite values")
    quaternion_norm = math.sqrt(
        sum(value * value for value in fields.orientation_xyzw)
    )
    if abs(quaternion_norm - 1.0) > 1e-3:
        fail(f"odometry quaternion norm is {quaternion_norm:.6f}, expected 1")
    return fields


def transform_values(transform: Any) -> tuple[float, ...]:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )


def transform_fields(transform: Any) -> GroundTruthOdomFields:
    values = transform_values(transform)
    fields = GroundTruthOdomFields(
        position_enu=values[:3],
        orientation_xyzw=values[3:],
        linear_velocity_enu=(0.0, 0.0, 0.0),
    )
    quaternion_norm = math.sqrt(
        sum(value * value for value in fields.orientation_xyzw)
    )
    if abs(quaternion_norm - 1.0) > 1e-3:
        fail(
            f"TF {transform.child_frame_id} quaternion norm is "
            f"{quaternion_norm:.6f}, expected 1"
        )
    return fields


def validate_markers(topic: str, message: MarkerArray) -> None:
    if not message.markers:
        fail(f"{topic} delivered an empty marker array")
    for marker in message.markers:
        if marker.header.frame_id != WORLD_FRAME:
            fail(
                f"{topic} marker {marker.ns}/{marker.id} frame "
                f"{marker.header.frame_id!r} is not {WORLD_FRAME!r}"
            )
        pose = marker.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            marker.scale.x,
            marker.scale.y,
            marker.scale.z,
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a,
            *(coordinate for point in marker.points for coordinate in (point.x, point.y, point.z)),
        )
        if not all(math.isfinite(value) for value in values):
            fail(f"{topic} marker {marker.ns}/{marker.id} has non-finite data")


def correlated_pose_errors(
    first: list[TimedOdom],
    second: list[TimedOdom],
) -> list[PoseComparison]:
    if not first or not second:
        fail("cannot compare empty odometry streams")

    def unique_by_stamp(samples: list[TimedOdom]) -> list[TimedOdom]:
        by_stamp = {sample.stamp_ns: sample for sample in samples}
        return sorted(by_stamp.values(), key=lambda sample: sample.stamp_ns)

    references = unique_by_stamp(first)[-MIN_ODOM_SAMPLES:]
    candidates = unique_by_stamp(second)
    if len(references) < MIN_ODOM_SAMPLES or len(candidates) < MIN_ODOM_SAMPLES:
        fail(
            "cannot compare fewer than "
            f"{MIN_ODOM_SAMPLES} unique odometry samples"
        )

    comparisons: list[PoseComparison] = []
    unused = list(candidates)
    for reference in references:
        candidate_index = min(
            range(len(unused)),
            key=lambda index: abs(
                reference.stamp_ns - unused[index].stamp_ns
            ),
        )
        candidate = unused.pop(candidate_index)
        first_fields = odom_fields(reference.message)
        second_fields = odom_fields(candidate.message)
        position_error_m, yaw_error_deg = odom_pose_errors(
            first_fields,
            second_fields,
        )
        comparisons.append(
            PoseComparison(
                time_error_s=(
                    abs(reference.stamp_ns - candidate.stamp_ns) / 1e9
                ),
                position_error_m=position_error_m,
                yaw_error_deg=yaw_error_deg,
                first=first_fields,
                second=second_fields,
            )
        )
    return comparisons


def assert_pose_match(
    label: str,
    first: list[TimedOdom],
    second: list[TimedOdom],
    *,
    max_time_error_s: float,
) -> None:
    comparisons = correlated_pose_errors(first, second)
    worst_position = max(
        comparisons,
        key=lambda comparison: comparison.position_error_m,
    )
    observed_max_time_error_s = max(
        comparison.time_error_s for comparison in comparisons
    )
    max_position_error_m = worst_position.position_error_m
    max_yaw_error_deg = max(
        comparison.yaw_error_deg for comparison in comparisons
    )
    print(
        f"{label} samples={len(comparisons)} "
        f"max_time_error_s={observed_max_time_error_s:.6f} "
        f"max_position_error_m={max_position_error_m:.6f} "
        f"max_yaw_error_deg={max_yaw_error_deg:.6f} "
        f"worst_positions={worst_position.first.position_enu},"
        f"{worst_position.second.position_enu}"
    )
    if any(
        comparison.time_error_s > max_time_error_s
        for comparison in comparisons
    ):
        fail(
            f"{label} has a sample without a sufficiently correlated "
            "odometry timestamp"
        )
    if max_position_error_m > MAX_POSITION_ERROR_M:
        fail(
            f"{label} position error {max_position_error_m:.6f} exceeds "
            f"{MAX_POSITION_ERROR_M:.2f} m"
        )
    if max_yaw_error_deg > MAX_YAW_ERROR_DEG:
        fail(
            f"{label} yaw error {max_yaw_error_deg:.6f} exceeds "
            f"{MAX_YAW_ERROR_DEG:.1f} degrees"
        )


def verify_publisher_qos(node: Any, topic: str, durability: Any) -> None:
    endpoints = node.get_publishers_info_by_topic(topic)
    if len(endpoints) != 1:
        fail(
            f"expected exactly one publisher for {topic}, found {len(endpoints)}"
        )
    offered = [endpoint.qos_profile.durability for endpoint in endpoints]
    if any(value != durability for value in offered):
        fail(f"{topic} publisher durability is {offered}, expected {durability}")
    print(
        f"qos topic={topic} durability={durability.name.lower()} publishers=1"
    )


def wait_until(
    node: Any,
    predicate: Callable[[], bool],
    deadline: float,
) -> None:
    while time.monotonic() < deadline and not predicate():
        rclpy.spin_once(node, timeout_sec=0.1)


def run(timeout_s: float) -> None:
    if timeout_s <= 0.0 or not math.isfinite(timeout_s):
        raise ValueError("timeout-s must be positive and finite")
    rclpy.init()
    node = rclpy.create_node("scn005_two_drone_topic_probe")
    odom_samples: dict[str, list[TimedOdom]] = defaultdict(list)
    tf_transforms: dict[str, Any] = {}
    marker_messages: dict[str, MarkerArray] = {}
    tf_stamps: set[int] = set()
    subscriptions: list[Any] = []
    deadline = time.monotonic() + timeout_s

    def record_odom(topic: str, message: Odometry) -> None:
        odom_fields(message)
        expected_frame, expected_child = EXPECTED_ODOM_FRAMES[topic]
        if (
            message.header.frame_id != expected_frame
            or message.child_frame_id != expected_child
        ):
            fail(
                f"{topic} frames are {message.header.frame_id!r} -> "
                f"{message.child_frame_id!r}, expected {expected_frame!r} -> "
                f"{expected_child!r}"
        )
        odom_samples[topic].append(
            TimedOdom(stamp_ns(message), message)
        )
        if len(odom_samples[topic]) > 100:
            del odom_samples[topic][:-100]

    def record_tf(message: TFMessage) -> None:
        for transform in message.transforms:
            values = transform_values(transform)
            if not all(math.isfinite(value) for value in values):
                fail(f"TF child {transform.child_frame_id} contains non-finite data")
            transform_fields(transform)
            tf_stamps.add(stamp_ns(transform))
            tf_transforms[transform.child_frame_id] = transform

    live_qos = QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    for topic in ODOM_TOPICS:
        subscriptions.append(
            node.create_subscription(
                Odometry,
                topic,
                lambda message, name=topic: record_odom(name, message),
                live_qos,
            )
        )
    subscriptions.append(
        node.create_subscription(TFMessage, TF_TOPIC, record_tf, live_qos)
    )

    def topics_ready() -> bool:
        advertised = dict(node.get_topic_names_and_types())
        return all(
            topic in advertised and expected_type in advertised[topic]
            for topic, expected_type in REQUIRED_TYPES.items()
        )

    wait_until(node, topics_ready, deadline)
    advertised = dict(node.get_topic_names_and_types())
    for topic, expected_type in REQUIRED_TYPES.items():
        if topic not in advertised:
            fail(f"required topic is not advertised: {topic}")
        if expected_type not in advertised[topic]:
            fail(
                f"{topic} types {advertised[topic]} do not include {expected_type}"
            )

    for topic in (*ODOM_TOPICS, TF_TOPIC):
        verify_publisher_qos(node, topic, DurabilityPolicy.VOLATILE)
    for topic in MARKER_TOPICS:
        verify_publisher_qos(node, topic, DurabilityPolicy.TRANSIENT_LOCAL)

    # Marker subscriptions are intentionally created only after publishers and
    # their topic types have been discovered.  They request transient-local
    # durability, and the endpoint checks above prove the writers offer it.
    marker_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    for topic in MARKER_TOPICS:
        subscriptions.append(
            node.create_subscription(
                MarkerArray,
                topic,
                lambda message, name=topic: marker_messages.__setitem__(name, message),
                marker_qos,
            )
        )

    required_tf_parents = {
        TARGET_BASE_LINK_FRAME: WORLD_FRAME,
        CHASER_BASE_LINK_FRAME: WORLD_FRAME,
        TARGET_FRONT_CAMERA_FRAME: TARGET_BASE_LINK_FRAME,
        CHASER_FRONT_CAMERA_FRAME: CHASER_BASE_LINK_FRAME,
    }
    required_tf_children = set(required_tf_parents)

    def data_ready() -> bool:
        return (
            all(
                len({sample.stamp_ns for sample in odom_samples[topic]})
                >= MIN_CORRELATION_BUFFER_SAMPLES
                for topic in ODOM_TOPICS
            )
            and all(topic in marker_messages for topic in MARKER_TOPICS)
            and required_tf_children.issubset(tf_transforms)
            and len(tf_stamps) >= MIN_ODOM_SAMPLES
        )

    wait_until(node, data_ready, deadline)
    for topic in ODOM_TOPICS:
        unique_stamps = len({sample.stamp_ns for sample in odom_samples[topic]})
        print(f"odom topic={topic} unique_samples={unique_stamps}")
        if unique_stamps < MIN_ODOM_SAMPLES:
            fail(
                f"{topic} delivered {unique_stamps} unique odometry samples; "
                f"expected at least {MIN_ODOM_SAMPLES}"
            )
    for topic in MARKER_TOPICS:
        if topic not in marker_messages:
            fail(f"late subscriber did not receive {topic}")
        validate_markers(topic, marker_messages[topic])

    scenario_namespaces = {
        marker.ns
        for marker in marker_messages[SCENARIO_MARKERS_TOPIC].markers
        if marker.action != Marker.DELETEALL
    }
    if "landing_pad" not in scenario_namespaces:
        fail("MAP landing_pad marker was not preserved on /sim/scenario_markers")
    drone_namespaces = {
        marker.ns for marker in marker_messages[DRONE_MARKERS_TOPIC].markers
    }
    for required_namespace in (
        "target_body",
        "chaser_body",
        "target_route",
        "desired_formation_point",
    ):
        if required_namespace not in drone_namespaces:
            fail(f"drone marker category is missing: {required_namespace}")

    active_drone_markers = [
        marker
        for marker in marker_messages[DRONE_MARKERS_TOPIC].markers
        if marker.action != Marker.DELETEALL
    ]
    route_marker = next(
        marker for marker in active_drone_markers if marker.ns == "target_route"
    )
    if route_marker.type != Marker.LINE_STRIP or len(route_marker.points) < 2:
        fail("target_route must be a non-empty LINE_STRIP")
    formation_marker = next(
        marker
        for marker in active_drone_markers
        if marker.ns == "desired_formation_point"
    )
    if formation_marker.type != Marker.SPHERE:
        fail("desired_formation_point must be a SPHERE")
    role_colors = {}
    for namespace in ("target_body", "chaser_body"):
        marker = next(
            candidate
            for candidate in active_drone_markers
            if candidate.ns == namespace
        )
        role_colors[namespace] = (
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a,
        )
    if role_colors["target_body"] == role_colors["chaser_body"]:
        fail("target and chaser markers must use distinct role colors")
    print(
        "markers "
        f"scenario={sorted(scenario_namespaces)} "
        f"drone={sorted(drone_namespaces)}"
    )

    for child_frame in required_tf_children:
        transform = tf_transforms.get(child_frame)
        if transform is None:
            fail(f"required TF child frame is missing: {child_frame}")
        expected_parent = required_tf_parents[child_frame]
        if transform.header.frame_id != expected_parent:
            fail(
                f"TF {child_frame} parent is {transform.header.frame_id!r}, "
                f"expected {expected_parent!r}"
            )
    for child_frame, topic in (
        (TARGET_BASE_LINK_FRAME, TARGET_ODOM_TOPIC),
        (CHASER_BASE_LINK_FRAME, CHASER_GROUND_TRUTH_ODOM_TOPIC),
    ):
        transform = tf_transforms[child_frame]
        transform_stamp = stamp_ns(transform)
        matching_odom = next(
            (
                sample
                for sample in odom_samples[topic]
                if sample.stamp_ns == transform_stamp
            ),
            None,
        )
        if matching_odom is None:
            fail(f"TF {child_frame} has no same-stamp {topic} sample")
        position_error_m, yaw_error_deg = odom_pose_errors(
            transform_fields(transform),
            odom_fields(matching_odom.message),
        )
        if position_error_m > 1e-9 or yaw_error_deg > 1e-9:
            fail(f"TF {child_frame} pose does not match {topic}")
    print(
        f"tf unique_samples={len(tf_stamps)} "
        f"children={sorted(tf_transforms)}"
    )

    # The two physics aliases carry an identical source state and ROS stamp.
    assert_pose_match(
        "ground_truth_alias",
        odom_samples[CHASER_GROUND_TRUTH_ODOM_TOPIC],
        odom_samples[GROUND_TRUTH_ODOM_TOPIC],
        max_time_error_s=0.0,
    )
    # ArduPilot SIM_STATE and direct physics ground truth are independent ROS
    # publishers, so correlate five one-to-one pairs by ROS header timestamp.
    assert_pose_match(
        "mavlink_chaser_alias",
        odom_samples[CHASER_GROUND_TRUTH_ODOM_TOPIC],
        odom_samples[DRONE_ODOM_TOPIC],
        max_time_error_s=0.25,
    )
    print("SCN-005 live topic probe completed successfully")

    del subscriptions
    node.destroy_node()
    rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=60.0)
    args = parser.parse_args()
    try:
        run(args.timeout_s)
    except Exception as exc:
        if rclpy.ok():
            rclpy.shutdown()
        print(f"FAIL: SCN-005 {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
