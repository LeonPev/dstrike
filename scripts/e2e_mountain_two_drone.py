#!/usr/bin/env python3
"""Bounded, measurement-based SCN-009 end-to-end verifier."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import MarkerArray
import yaml

from formation_probe import body_offset_world, distance, point_to_route_error, quaternion_yaw


class ProbeFailure(RuntimeError):
    pass


def terrain_height(heights, size_m, x, y):
    radius = size_m / 2.0
    row = (radius - y) * (heights.shape[0] - 1) / size_m
    col = (x + radius) * (heights.shape[1] - 1) / size_m
    row = min(max(row, 0.0), heights.shape[0] - 1.0)
    col = min(max(col, 0.0), heights.shape[1] - 1.0)
    r0, c0 = int(math.floor(row)), int(math.floor(col))
    r1, c1 = min(r0 + 1, heights.shape[0] - 1), min(c0 + 1, heights.shape[1] - 1)
    ty, tx = row - r0, col - c0
    return float((1 - ty) * ((1 - tx) * heights[r0, c0] + tx * heights[r0, c1]) + ty * ((1 - tx) * heights[r1, c0] + tx * heights[r1, c1]))


def enabled(value):
    return value.strip().lower() not in ("0", "false", "no", "off", "")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float)
    args = parser.parse_args()
    root = Path("/workspace")
    scenario = yaml.safe_load((root / "config/scenarios/mountain_two_drones.yaml").read_text())
    map_config = yaml.safe_load((root / "models/scenarios/mountain_map/scenario.yaml").read_text())
    safety, target_cfg, chaser_cfg = scenario["safety"], scenario["target_drone"], scenario["chaser_drone"]
    timeout = args.timeout_s or float(safety["mission_timeout_s"])
    size_m = float(map_config["terrain"]["size_m"])
    heights = np.load(root / "models/scenarios/mountain_map/heightfield_m.npy")
    heading = math.radians(float(target_cfg["heading_deg"]))
    start_xy = tuple(map(float, target_cfg["start_xy_m"]))
    offset = tuple(map(float, chaser_cfg["formation_offset_target_body_m"]))
    threshold = float(safety["formation_error_threshold_m"])
    hold_required = float(safety["formation_hold_required_s"])
    min_clearance = float(safety["min_terrain_clearance_m"])
    min_separation = float(safety["min_drone_separation_m"])
    expected_target_agl = float(target_cfg["altitude_agl_m"])
    cameras_required = enabled(os.environ.get("CAMERA_ENABLED", "1"))

    rclpy.init()
    node = Node("scn009_e2e_probe")
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = None
    state_history = []
    chaser = target = None
    initial_chaser_z = None
    markers_seen = False
    cameras_seen = set()
    formation_message = None
    samples = 0
    min_observed_separation = float("inf")
    max_cross_track = 0.0
    max_target_agl_error = 0.0
    min_pre_clearance = float("inf")
    min_post_clearance = float("inf")
    clearance_enforced = False
    first_error = None
    best_error = float("inf")
    hold_started = None
    held_for = 0.0

    def on_state(message):
        nonlocal state
        state = message.data
        if not state_history or state_history[-1] != state:
            state_history.append(state)
            print(f"state={state}", flush=True)

    def on_chaser(message):
        nonlocal chaser, initial_chaser_z
        p = message.pose.pose.position
        chaser = (float(p.x), float(p.y), float(p.z))
        if initial_chaser_z is None:
            initial_chaser_z = chaser[2]

    def on_target(message):
        nonlocal target
        p = message.pose.pose.position
        target = ((float(p.x), float(p.y), float(p.z)), quaternion_yaw(message.pose.pose.orientation))

    def on_markers(_message):
        nonlocal markers_seen
        markers_seen = True

    def on_error(message):
        nonlocal formation_message
        formation_message = float(message.data)

    node.create_subscription(String, "/scenario/state", on_state, state_qos)
    node.create_subscription(Odometry, "/drone/odom", on_chaser, 20)
    node.create_subscription(Odometry, "/sim/target/odom", on_target, 20)
    node.create_subscription(MarkerArray, "/sim/scenario_markers", on_markers, 10)
    node.create_subscription(Float32, "/scenario/formation_error_m", on_error, 20)
    if cameras_required:
        for topic in ("/sim/target/camera/front/image_raw", "/sim/chaser/camera/front/image_raw"):
            node.create_subscription(Image, topic, lambda _m, name=topic: cameras_seen.add(name), 5)
    start_pub = node.create_publisher(Bool, "/scenario/start", 10)

    try:
        ready_deadline = time.monotonic() + 45.0
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if chaser and target and markers_seen and state is not None and (not cameras_required or len(cameras_seen) == 2):
                break
        else:
            raise ProbeFailure(f"startup telemetry incomplete: chaser={bool(chaser)} target={bool(target)} markers={markers_seen} state={state} cameras={len(cameras_seen)}/2")

        # Publish across several controller ticks so discovery races cannot lose start.
        for _ in range(10):
            start_pub.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.1)
        deadline = time.monotonic() + timeout
        last_report = 0.0
        while time.monotonic() < deadline and state not in ("COMPLETE", "ABORT"):
            rclpy.spin_once(node, timeout_sec=0.05)
            if not chaser or not target:
                continue
            now = time.monotonic()
            target_xyz, target_yaw = target
            separation = distance(chaser, target_xyz)
            desired = body_offset_world(target_xyz, target_yaw, offset)
            error = distance(chaser, desired)
            target_ground = terrain_height(heights, size_m, target_xyz[0], target_xyz[1])
            chaser_ground = terrain_height(heights, size_m, chaser[0], chaser[1])
            clearance = chaser[2] - chaser_ground
            cross_track = point_to_route_error(target_xyz, start_xy, heading)
            target_agl_error = abs((target_xyz[2] - target_ground) - expected_target_agl)
            samples += 1
            min_observed_separation = min(min_observed_separation, separation)
            max_cross_track = max(max_cross_track, cross_track)
            max_target_agl_error = max(max_target_agl_error, target_agl_error)
            if clearance_enforced or clearance >= min_clearance or state in ("INTERCEPT", "FORMATION_HOLD", "COMPLETE"):
                clearance_enforced = True
                min_post_clearance = min(min_post_clearance, clearance)
            else:
                min_pre_clearance = min(min_pre_clearance, clearance)
            if state in ("INTERCEPT", "FORMATION_HOLD", "COMPLETE"):
                first_error = error if first_error is None else first_error
                best_error = min(best_error, error)
            if error <= threshold:
                hold_started = now if hold_started is None else hold_started
                held_for = now - hold_started
            else:
                hold_started, held_for = None, 0.0
            if separation < min_separation:
                raise ProbeFailure(f"separation violation: {separation:.3f}m < {min_separation:.3f}m")
            if clearance_enforced and clearance < min_clearance:
                raise ProbeFailure(f"post-enforcement clearance violation: {clearance:.3f}m < {min_clearance:.3f}m")
            if now - last_report >= 2.0:
                print(f"metrics formation={error:.2f}m separation={separation:.2f}m clearance={clearance:.2f}m hold={held_for:.1f}s", flush=True)
                last_report = now

        if state == "ABORT":
            raise ProbeFailure("controller entered ABORT")
        if state != "COMPLETE":
            raise ProbeFailure(f"mission did not complete within {timeout:g}s; final state={state}")
        if not chaser or not target or initial_chaser_z is None:
            raise ProbeFailure("final telemetry unavailable")
        final_desired = body_offset_world(target[0], target[1], offset)
        final_error = distance(chaser, final_desired)
        altitude_gain = chaser[2] - initial_chaser_z
        checks = [
            (samples > 0, "no synchronized samples"),
            (max_cross_track <= 0.50, f"target cross-track error {max_cross_track:.3f}m > 0.50m"),
            (max_target_agl_error <= 0.75, f"target AGL error {max_target_agl_error:.3f}m > 0.75m"),
            (altitude_gain >= 5.0, f"chaser altitude gain {altitude_gain:.3f}m < 5.0m"),
            (held_for >= hold_required, f"continuous formation hold {held_for:.2f}s < {hold_required:.2f}s"),
            (final_error <= threshold, f"final formation error {final_error:.3f}m > {threshold:.3f}m"),
            (first_error is not None and best_error < first_error, "formation error did not decrease"),
            (formation_message is not None, "formation error topic was not observed"),
        ]
        for passed, reason in checks:
            if not passed:
                raise ProbeFailure(reason)
        print(f"state-history={','.join(state_history)}")
        print(f"target-cross-track-max-m={max_cross_track:.3f}")
        print(f"target-agl-error-max-m={max_target_agl_error:.3f}")
        print(f"chaser-altitude-gain-m={altitude_gain:.3f}")
        print(f"formation-error-final-m={final_error:.3f} continuous-hold-s={held_for:.2f}")
        print(f"terrain-clearance-min-pre-m={min_pre_clearance:.3f} min-post-m={min_post_clearance:.3f}")
        print(f"minimum-observed-separation-m={min_observed_separation:.3f}")
        return 0
    except ProbeFailure as exc:
        print(f"minimum-observed-separation-m={min_observed_separation:.3f}", file=sys.stderr)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
