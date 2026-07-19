#!/usr/bin/env python3
"""Late-subscriber content verification for MAP-005 static marker topics."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


TOPICS = (
    "/sim/terrain_markers",
    "/sim/nature_markers",
    "/sim/scenario_markers",
    "/sim/drone_markers",
)


def fail(message: str) -> None:
    raise RuntimeError(message)


def main() -> int:
    asset_dir = Path("/workspace/models/scenarios/mountain_map")
    sys.path.insert(0, "/workspace")
    from src.physics.scenario_config import load_mountain_map_config
    from src.physics.ros_visualization import terrain_mesh, terrain_mesh_height_at

    config = load_mountain_map_config(asset_dir / "scenario.yaml")
    heights = np.load(asset_dir / "heightfield_m.npy")
    expected_mesh = terrain_mesh(
        heights, config.terrain.size_m, config.terrain.rviz_decimation
    )
    nature = json.loads((asset_dir / "nature.json").read_text(encoding="utf-8"))

    # The physics publisher has already emitted its one-shot static arrays before
    # this process starts. Receiving them proves transient-local late delivery.
    rclpy.init()
    node = rclpy.create_node("map005_marker_probe")
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    received: dict[str, MarkerArray] = {}
    subscriptions = [
        node.create_subscription(
            MarkerArray, topic, lambda message, name=topic: received.setdefault(name, message), qos
        )
        for topic in TOPICS
    ]
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline and len(received) < len(TOPICS):
        rclpy.spin_once(node, timeout_sec=0.2)

    advertised = dict(node.get_topic_names_and_types())
    for topic in TOPICS:
        if topic not in advertised:
            fail(f"topic does not exist: {topic}")
        if "visualization_msgs/msg/MarkerArray" not in advertised[topic]:
            fail(f"wrong message type for {topic}: {advertised[topic]}")
        if topic not in received:
            fail(f"late subscriber did not receive transient-local data: {topic}")

    terrain_markers = received[TOPICS[0]].markers
    triangles = [marker for marker in terrain_markers if marker.type == Marker.TRIANGLE_LIST]
    if not triangles:
        fail("terrain has no TRIANGLE_LIST marker")
    points = [point for marker in triangles for point in marker.points]
    if any(marker.header.frame_id != "world" for marker in terrain_markers):
        fail("terrain marker frame is not world")
    coordinates = np.array([(p.x, p.y, p.z) for p in points], dtype=np.float64)
    if coordinates.size == 0 or not np.isfinite(coordinates).all():
        fail("terrain vertices are empty or non-finite")
    expected = np.asarray(expected_mesh.points_xyz)
    tolerances = (0.01, 0.01, 0.05)
    for axis, name, tolerance in zip(range(3), "xyz", tolerances):
        for reducer, label in ((np.min, "min"), (np.max, "max")):
            actual_bound = float(reducer(coordinates[:, axis]))
            expected_bound = float(reducer(expected[:, axis]))
            if abs(actual_bound - expected_bound) > tolerance:
                fail(f"terrain {name} {label} {actual_bound} != {expected_bound}")
    if len(points) != len(expected_mesh.points_xyz):
        fail(f"terrain point count {len(points)} != {len(expected_mesh.points_xyz)}")
    if len(points) // 3 != expected_mesh.triangle_count:
        fail("terrain triangle count does not match configured decimation")
    for index in range(0, len(coordinates), 3):
        normal = np.cross(
            coordinates[index + 1] - coordinates[index],
            coordinates[index + 2] - coordinates[index],
        )
        if normal[2] <= 0.0:
            fail(f"terrain triangle {index // 3} is not upward-facing")
    if config.terrain.resolution == 129 and config.terrain.rviz_decimation == 4:
        if (expected_mesh.triangle_count, len(points)) != (2048, 6144):
            fail("default terrain must contain exactly 2048 triangles and 6144 points")

    nature_markers = received[TOPICS[1]].markers
    if any(marker.header.frame_id != "world" for marker in nature_markers):
        fail("nature marker frame is not world")
    tree_markers = [marker for marker in nature_markers if marker.ns == "trees"]
    rock_markers = [marker for marker in nature_markers if marker.ns == "rocks"]
    if len(tree_markers) != len(nature["trees"]) or len(rock_markers) != len(nature["rocks"]):
        fail("nature marker counts do not match nature.json")
    if (config.nature.trees, config.nature.rocks) == (80, 35):
        if (len(tree_markers), len(rock_markers)) != (80, 35):
            fail("default nature marker counts must be exactly 80 trees and 35 rocks")
    for marker in tree_markers + rock_markers:
        distance = math.hypot(
            marker.pose.position.x - config.spawn.x_m,
            marker.pose.position.y - config.spawn.y_m,
        )
        if distance < config.nature.avoid_spawn_radius_m:
            fail(f"nature marker {marker.ns}/{marker.id} is inside spawn exclusion zone")
        expected_ground_z = terrain_mesh_height_at(
            expected_mesh, marker.pose.position.x, marker.pose.position.y
        )
        marker_base_z = marker.pose.position.z - marker.scale.z / 2.0
        if abs(marker_base_z - expected_ground_z) > 0.01:
            fail(
                f"nature marker {marker.ns}/{marker.id} base {marker_base_z} "
                f"does not touch displayed terrain {expected_ground_z}"
            )

    del subscriptions
    node.destroy_node()
    rclpy.shutdown()
    print(
        f"terrain triangles={expected_mesh.triangle_count} points={len(points)} "
        f"trees={len(tree_markers)} rocks={len(rock_markers)}"
    )
    print("PASS: MAP-005 RViz terrain and nature markers are content-verified")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"FAIL: MAP-005 {exc}", file=sys.stderr)
        raise SystemExit(1)
