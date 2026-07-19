#!/usr/bin/env python3
"""Run a bounded mountain-map vertical-flight regression through ROS 2."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import os
import statistics
import sys
import time
from typing import Any, Callable


PASS_LINE = "PASS: mountain vertical takeoff, hover, up/down, and landing verified"


class ProbeFailure(RuntimeError):
    """Raised when a bounded live flight assertion fails."""


@dataclass(frozen=True)
class GroundTruthSample:
    received_s: float
    x_m: float
    y_m: float
    z_m: float
    vz_m_s: float
    tilt_deg: float


@dataclass(frozen=True)
class WindowStats:
    count: int
    duration_s: float
    median_x_m: float
    median_y_m: float
    median_z_m: float
    z_span_m: float
    max_abs_vz_m_s: float
    max_tilt_deg: float
    max_xy_drift_m: float


def quaternion_tilt_deg(x: float, y: float, z: float, w: float) -> float:
    """Return the angle between the body and world up axes."""

    if not all(math.isfinite(value) for value in (x, y, z, w)):
        return float("nan")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return 180.0
    x /= norm
    y /= norm
    body_up_world_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, body_up_world_z))))


def window_stats(
    samples: list[GroundTruthSample],
    anchor_xy: tuple[float, float],
) -> WindowStats:
    if not samples:
        raise ValueError("at least one ground-truth sample is required")
    if not all(
        math.isfinite(value)
        for sample in samples
        for value in (
            sample.received_s,
            sample.x_m,
            sample.y_m,
            sample.z_m,
            sample.vz_m_s,
            sample.tilt_deg,
        )
    ):
        raise ValueError("ground-truth samples must be finite")
    anchor_x, anchor_y = anchor_xy
    z_values = [sample.z_m for sample in samples]
    return WindowStats(
        count=len(samples),
        duration_s=samples[-1].received_s - samples[0].received_s,
        median_x_m=float(statistics.median(sample.x_m for sample in samples)),
        median_y_m=float(statistics.median(sample.y_m for sample in samples)),
        median_z_m=float(statistics.median(z_values)),
        z_span_m=max(z_values) - min(z_values),
        max_abs_vz_m_s=max(abs(sample.vz_m_s) for sample in samples),
        max_tilt_deg=max(sample.tilt_deg for sample in samples),
        max_xy_drift_m=max(
            math.hypot(sample.x_m - anchor_x, sample.y_m - anchor_y)
            for sample in samples
        ),
    )


def heartbeat_has_local_position_ned(
    heartbeat: dict[str, Any] | None,
) -> bool:
    return bool(heartbeat and heartbeat.get("has_local_position_ned"))


def landed_window_is_stable(
    samples: list[GroundTruthSample],
    ground_xyz: tuple[float, float, float],
    now_s: float,
    minimum_duration_s: float,
    telemetry_max_age_s: float,
    z_tolerance_m: float,
    max_abs_vz_m_s: float,
    max_xy_error_m: float,
    max_tilt_deg: float,
) -> bool:
    if not samples:
        return False
    try:
        window_stats(samples, ground_xyz[:2])
    except ValueError:
        return False
    if samples[-1].received_s - samples[0].received_s < minimum_duration_s:
        return False
    if now_s - samples[-1].received_s > telemetry_max_age_s:
        return False
    ground_x, ground_y, ground_z = ground_xyz
    return all(
        abs(sample.z_m - ground_z) <= z_tolerance_m
        and abs(sample.vz_m_s) <= max_abs_vz_m_s
        and math.hypot(sample.x_m - ground_x, sample.y_m - ground_y)
        <= max_xy_error_m
        and sample.tilt_deg <= max_tilt_deg
        for sample in samples
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Arm and take off over mountain_map, verify stable hover and "
            "vertical velocity control from MuJoCo ground truth, then land."
        )
    )
    readiness = parser.add_argument_group("readiness and ground settling")
    readiness.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=180.0,
        help="Seconds to wait for the bridged MAVLink heartbeat.",
    )
    readiness.add_argument(
        "--ground-truth-timeout",
        type=float,
        default=120.0,
        help="Seconds to wait for MuJoCo ground-truth odometry.",
    )
    readiness.add_argument(
        "--local-position-timeout",
        type=float,
        default=120.0,
        help=(
            "Seconds to wait for the bridge to confirm a real ArduPilot "
            "LOCAL_POSITION_NED message before GUIDED arming."
        ),
    )
    readiness.add_argument(
        "--endpoint-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for all required ROS publishers/subscribers.",
    )
    readiness.add_argument(
        "--telemetry-max-age",
        type=float,
        default=1.0,
        help="Maximum allowed age of heartbeat or ground-truth data in flight.",
    )
    readiness.add_argument(
        "--settle-timeout",
        type=float,
        default=30.0,
        help="Seconds allowed for the vehicle to settle before arming.",
    )
    readiness.add_argument(
        "--settle-window",
        type=float,
        default=3.0,
        help="Continuous stable-ground window required before arming.",
    )
    readiness.add_argument(
        "--settle-z-span-m",
        type=float,
        default=0.15,
        help="Maximum ground-truth Z span in the settle window.",
    )
    readiness.add_argument(
        "--settle-max-vz-m-s",
        type=float,
        default=0.20,
        help="Maximum absolute vertical speed in the settle window.",
    )

    flight = parser.add_argument_group("takeoff and flight assertions")
    flight.add_argument(
        "--arm-timeout",
        type=float,
        default=20.0,
        help="Seconds to wait for armed=true after the repeated arm burst.",
    )
    flight.add_argument(
        "--takeoff-altitude-m",
        type=float,
        default=3.0,
        help="Relative takeoff altitude sent to /drone/takeoff_alt_m.",
    )
    flight.add_argument(
        "--takeoff-min-gain-m",
        type=float,
        default=2.0,
        help="Minimum ground-truth Z gain relative to settled ground.",
    )
    flight.add_argument(
        "--takeoff-timeout",
        type=float,
        default=30.0,
        help="Seconds allowed to reach the minimum relative takeoff gain.",
    )
    flight.add_argument(
        "--hover-stabilize",
        type=float,
        default=3.0,
        help="Unmeasured stabilization seconds before the hover window.",
    )
    flight.add_argument(
        "--hover-duration",
        type=float,
        default=20.0,
        help="Seconds of stable armed hover required.",
    )
    flight.add_argument(
        "--max-hover-z-span-m",
        type=float,
        default=0.60,
        help="Maximum Z range across the measured hover window.",
    )
    flight.add_argument(
        "--max-hover-vz-m-s",
        type=float,
        default=0.40,
        help="Maximum absolute vertical speed during measured hover.",
    )
    flight.add_argument(
        "--max-tilt-deg",
        type=float,
        default=15.0,
        help="Maximum body-up tilt during active flight.",
    )
    flight.add_argument(
        "--max-xy-drift-m",
        type=float,
        default=0.75,
        help="Maximum horizontal distance from the settled launch point.",
    )
    flight.add_argument(
        "--min-flight-clearance-m",
        type=float,
        default=0.75,
        help="Minimum Z above settled ground after takeoff is reached.",
    )

    vertical = parser.add_argument_group("vertical velocity response")
    vertical.add_argument(
        "--vertical-speed-m-s",
        type=float,
        default=0.50,
        help="Magnitude of continuous ROS ENU cmd_vel Z commands.",
    )
    vertical.add_argument(
        "--vertical-command-duration",
        type=float,
        default=4.0,
        help="Seconds to publish each up/down velocity command.",
    )
    vertical.add_argument(
        "--vertical-min-delta-m",
        type=float,
        default=1.0,
        help="Minimum directed Z response required after each command.",
    )
    vertical.add_argument(
        "--zero-command-duration",
        type=float,
        default=1.0,
        help="Seconds to publish zero velocity after each phase.",
    )
    vertical.add_argument(
        "--post-command-hold",
        type=float,
        default=3.0,
        help="Seconds to hold and observe after each vertical command.",
    )
    vertical.add_argument(
        "--hold-window",
        type=float,
        default=1.5,
        help="Trailing post-command seconds used for response measurement.",
    )
    vertical.add_argument(
        "--max-hold-z-span-m",
        type=float,
        default=0.60,
        help="Maximum Z range in a trailing hold window.",
    )
    vertical.add_argument(
        "--max-hold-vz-m-s",
        type=float,
        default=0.40,
        help="Maximum absolute vertical speed in a trailing hold window.",
    )
    vertical.add_argument(
        "--return-tolerance-m",
        type=float,
        default=0.75,
        help="Allowed post-down difference from the pre-up hover Z.",
    )

    commands = parser.add_argument_group("command publication and cleanup")
    commands.add_argument(
        "--command-rate-hz",
        type=float,
        default=10.0,
        help="Publication rate for repeated flight commands.",
    )
    commands.add_argument(
        "--command-duration",
        type=float,
        default=1.0,
        help="Seconds to repeat takeoff, land, and disarm commands.",
    )
    commands.add_argument(
        "--land-timeout",
        type=float,
        default=30.0,
        help="Seconds allowed to return to the settled ground height.",
    )
    commands.add_argument(
        "--land-stable-window",
        type=float,
        default=1.5,
        help="Continuous ground-stable window required before disarm.",
    )
    commands.add_argument(
        "--land-z-tolerance-m",
        type=float,
        default=0.15,
        help="Allowed distance from settled ground Z when landed.",
    )
    commands.add_argument(
        "--land-max-vz-m-s",
        type=float,
        default=0.20,
        help="Maximum absolute vertical speed in the landed window.",
    )
    commands.add_argument(
        "--land-max-xy-error-m",
        type=float,
        default=0.90,
        help="Maximum landed horizontal distance from the launch-pad center.",
    )
    commands.add_argument(
        "--land-max-tilt-deg",
        type=float,
        default=5.0,
        help="Maximum body-up tilt accepted before ground-gated force disarm.",
    )
    commands.add_argument(
        "--disarm-timeout",
        type=float,
        default=15.0,
        help="Seconds to wait for armed=false during cleanup.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    positive_fields = (
        "heartbeat_timeout",
        "ground_truth_timeout",
        "local_position_timeout",
        "endpoint_timeout",
        "telemetry_max_age",
        "settle_timeout",
        "settle_window",
        "settle_z_span_m",
        "settle_max_vz_m_s",
        "arm_timeout",
        "takeoff_altitude_m",
        "takeoff_min_gain_m",
        "takeoff_timeout",
        "hover_duration",
        "max_hover_z_span_m",
        "max_hover_vz_m_s",
        "max_tilt_deg",
        "max_xy_drift_m",
        "min_flight_clearance_m",
        "vertical_speed_m_s",
        "vertical_command_duration",
        "vertical_min_delta_m",
        "zero_command_duration",
        "post_command_hold",
        "hold_window",
        "max_hold_z_span_m",
        "max_hold_vz_m_s",
        "return_tolerance_m",
        "command_rate_hz",
        "command_duration",
        "land_timeout",
        "land_stable_window",
        "land_z_tolerance_m",
        "land_max_vz_m_s",
        "land_max_xy_error_m",
        "land_max_tilt_deg",
        "disarm_timeout",
    )
    for field in positive_fields:
        if float(getattr(args, field)) <= 0.0:
            raise ProbeFailure(f"--{field.replace('_', '-')} must be positive")
    if args.hover_stabilize < 0.0:
        raise ProbeFailure("--hover-stabilize must be non-negative")
    if args.takeoff_min_gain_m >= args.takeoff_altitude_m:
        raise ProbeFailure(
            "--takeoff-min-gain-m must be less than --takeoff-altitude-m"
        )
    if args.hold_window > args.post_command_hold:
        raise ProbeFailure("--hold-window must not exceed --post-command-hold")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except ProbeFailure as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2

    scenario_name = os.environ.get("SCENARIO", "").strip().lower().replace("-", "_")
    if scenario_name != "mountain_map":
        print(
            "FAIL: SCENARIO must be mountain_map; start with `make up-map-viz`",
            file=sys.stderr,
        )
        return 2

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from std_msgs.msg import Bool, Float32, String
    except ImportError as exc:
        print(f"FAIL: ROS 2 import failed: {exc}", file=sys.stderr)
        return 2

    class MountainFlightProbe(Node):
        def __init__(self) -> None:
            super().__init__("mountain_vertical_flight_probe")
            self.heartbeat: dict[str, Any] | None = None
            self.heartbeat_received_s: float | None = None
            self.samples: deque[GroundTruthSample] = deque(maxlen=30_000)
            self.create_subscription(
                String, "/drone/heartbeat", self._on_heartbeat, 10
            )
            self.create_subscription(
                Odometry,
                "/sim/ground_truth/odom",
                self._on_ground_truth,
                10,
            )
            self.arm_pub = self.create_publisher(Bool, "/drone/arm", 10)
            self.force_disarm_pub = self.create_publisher(
                Bool,
                "/drone/force_disarm",
                10,
            )
            self.takeoff_pub = self.create_publisher(
                Float32, "/drone/takeoff_alt_m", 10
            )
            self.land_pub = self.create_publisher(Bool, "/drone/land", 10)
            self.velocity_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)

        def _on_heartbeat(self, message: String) -> None:
            try:
                heartbeat = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if isinstance(heartbeat, dict):
                self.heartbeat = heartbeat
                self.heartbeat_received_s = time.monotonic()

        def _on_ground_truth(self, message: Odometry) -> None:
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            self.samples.append(
                GroundTruthSample(
                    received_s=time.monotonic(),
                    x_m=float(position.x),
                    y_m=float(position.y),
                    z_m=float(position.z),
                    vz_m_s=float(message.twist.twist.linear.z),
                    tilt_deg=quaternion_tilt_deg(
                        float(orientation.x),
                        float(orientation.y),
                        float(orientation.z),
                        float(orientation.w),
                    ),
                )
            )

        @property
        def armed(self) -> bool:
            return bool(self.heartbeat and self.heartbeat.get("armed"))

        @property
        def local_position_ready(self) -> bool:
            return heartbeat_has_local_position_ned(self.heartbeat)

        @property
        def latest(self) -> GroundTruthSample | None:
            return self.samples[-1] if self.samples else None

        def samples_since(self, start_s: float) -> list[GroundTruthSample]:
            return [sample for sample in self.samples if sample.received_s >= start_s]

        def recent_samples(self, duration_s: float) -> list[GroundTruthSample]:
            if not self.samples:
                return []
            threshold_s = self.samples[-1].received_s - duration_s
            return [
                sample for sample in self.samples if sample.received_s >= threshold_s
            ]

        def missing_endpoints(self) -> list[str]:
            missing: list[str] = []
            for topic in (
                "/drone/heartbeat",
                "/sim/ground_truth/odom",
            ):
                if self.count_publishers(topic) < 1:
                    missing.append(f"publisher:{topic}")
            for topic in (
                "/drone/arm",
                "/drone/force_disarm",
                "/drone/takeoff_alt_m",
                "/drone/land",
                "/drone/cmd_vel",
            ):
                if self.count_subscribers(topic) < 1:
                    missing.append(f"subscriber:{topic}")
            return missing

    def spin_until(timeout_s: float, predicate: Callable[[], bool]) -> bool:
        deadline_s = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline_s:
            if predicate():
                return True
            remaining_s = deadline_s - time.monotonic()
            rclpy.spin_once(node, timeout_sec=max(0.01, min(0.05, remaining_s)))
        return predicate()

    def publish_for(publisher: Any, message: Any, duration_s: float) -> None:
        deadline_s = time.monotonic() + duration_s
        period_s = 1.0 / args.command_rate_hz
        next_publish_s = 0.0
        while rclpy.ok() and time.monotonic() < deadline_s:
            now_s = time.monotonic()
            if now_s >= next_publish_s:
                publisher.publish(message)
                next_publish_s = now_s + period_s
            remaining_s = deadline_s - now_s
            until_publish_s = max(0.0, next_publish_s - now_s)
            rclpy.spin_once(
                node,
                timeout_sec=max(
                    0.001, min(0.05, remaining_s, until_publish_s or 0.001)
                ),
            )

    def require_flight_safe(
        ground_xyz: tuple[float, float, float],
        require_clearance: bool = True,
    ) -> None:
        sample = node.latest
        if sample is None:
            raise ProbeFailure("ground-truth odometry stopped")
        if not all(
            math.isfinite(value)
            for value in (
                sample.received_s,
                sample.x_m,
                sample.y_m,
                sample.z_m,
                sample.vz_m_s,
                sample.tilt_deg,
            )
        ):
            raise ProbeFailure("ground-truth odometry contains non-finite values")
        now_s = time.monotonic()
        if now_s - sample.received_s > args.telemetry_max_age:
            raise ProbeFailure("ground-truth odometry became stale")
        if (
            node.heartbeat_received_s is None
            or now_s - node.heartbeat_received_s > args.telemetry_max_age
        ):
            raise ProbeFailure("heartbeat became stale")
        ground_x, ground_y, ground_z = ground_xyz
        if not node.armed:
            raise ProbeFailure("vehicle disarmed during active flight")
        xy_drift_m = math.hypot(sample.x_m - ground_x, sample.y_m - ground_y)
        if xy_drift_m > args.max_xy_drift_m:
            raise ProbeFailure(
                f"horizontal drift {xy_drift_m:.3f}m exceeds "
                f"{args.max_xy_drift_m:.3f}m"
            )
        if sample.tilt_deg > args.max_tilt_deg:
            raise ProbeFailure(
                f"tilt {sample.tilt_deg:.2f}deg exceeds "
                f"{args.max_tilt_deg:.2f}deg"
            )
        clearance_m = sample.z_m - ground_z
        if require_clearance and clearance_m < args.min_flight_clearance_m:
            raise ProbeFailure(
                f"clearance {clearance_m:.3f}m fell below "
                f"{args.min_flight_clearance_m:.3f}m"
            )

    def monitor_flight(
        duration_s: float,
        ground_xyz: tuple[float, float, float],
    ) -> list[GroundTruthSample]:
        start_s = time.monotonic()
        deadline_s = start_s + duration_s
        while rclpy.ok() and time.monotonic() < deadline_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            require_flight_safe(ground_xyz)
        samples = node.samples_since(start_s)
        if not samples or samples[-1].received_s - samples[0].received_s < 0.8 * duration_s:
            raise ProbeFailure("insufficient ground-truth coverage during flight phase")
        return samples

    def settle_ground() -> tuple[float, float, float]:
        print(
            f"[settle] waiting for {args.settle_window:g}s stable ground window"
        )
        deadline_s = time.monotonic() + args.settle_timeout
        last_stats: WindowStats | None = None
        while rclpy.ok() and time.monotonic() < deadline_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            recent = node.recent_samples(args.settle_window)
            if not recent:
                continue
            stats = window_stats(recent, (recent[0].x_m, recent[0].y_m))
            last_stats = stats
            if stats.duration_s < 0.8 * args.settle_window:
                continue
            if (
                stats.z_span_m <= args.settle_z_span_m
                and stats.max_abs_vz_m_s <= args.settle_max_vz_m_s
            ):
                print(
                    "[settle] ground="
                    f"({stats.median_x_m:.3f}, {stats.median_y_m:.3f}, "
                    f"{stats.median_z_m:.3f}) z_span={stats.z_span_m:.3f}m "
                    f"max_vz={stats.max_abs_vz_m_s:.3f}m/s"
                )
                return (
                    stats.median_x_m,
                    stats.median_y_m,
                    stats.median_z_m,
                )
        detail = "no ground-truth window"
        if last_stats is not None:
            detail = (
                f"z_span={last_stats.z_span_m:.3f}m "
                f"max_vz={last_stats.max_abs_vz_m_s:.3f}m/s"
            )
        raise ProbeFailure(f"ground did not settle within timeout ({detail})")

    def arm() -> None:
        print(f"[arm] requesting arm; timeout={args.arm_timeout:g}s")
        start_s = time.monotonic()
        publish_for(node.arm_pub, Bool(data=True), args.command_duration)
        remaining_s = max(0.01, args.arm_timeout - (time.monotonic() - start_s))
        if not spin_until(remaining_s, lambda: node.armed):
            raise ProbeFailure("timed out waiting for armed=true")
        heartbeat = node.heartbeat or {}
        print(f"[arm] armed=true mode={heartbeat.get('mode', 'UNKNOWN')}")

    def takeoff(ground_xyz: tuple[float, float, float]) -> None:
        print(
            f"[takeoff] command={args.takeoff_altitude_m:g}m; "
            f"required relative gain={args.takeoff_min_gain_m:g}m"
        )
        publish_for(
            node.takeoff_pub,
            Float32(data=float(args.takeoff_altitude_m)),
            args.command_duration,
        )
        deadline_s = time.monotonic() + args.takeoff_timeout
        while rclpy.ok() and time.monotonic() < deadline_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            require_flight_safe(ground_xyz, require_clearance=False)
            sample = node.latest
            if sample is not None:
                gain_m = sample.z_m - ground_xyz[2]
                if gain_m >= args.takeoff_min_gain_m:
                    print(f"[takeoff] relative gain={gain_m:.3f}m")
                    return
        sample = node.latest
        gain_m = float("nan") if sample is None else sample.z_m - ground_xyz[2]
        raise ProbeFailure(
            f"takeoff gain reached only {gain_m:.3f}m within timeout"
        )

    def verify_hover(ground_xyz: tuple[float, float, float]) -> float:
        if args.hover_stabilize > 0.0:
            print(f"[hover] stabilizing for {args.hover_stabilize:g}s")
            monitor_flight(args.hover_stabilize, ground_xyz)
        print(f"[hover] verifying stable hover for {args.hover_duration:g}s")
        samples = monitor_flight(args.hover_duration, ground_xyz)
        stats = window_stats(samples, ground_xyz[:2])
        if stats.z_span_m > args.max_hover_z_span_m:
            raise ProbeFailure(
                f"hover z span {stats.z_span_m:.3f}m exceeds "
                f"{args.max_hover_z_span_m:.3f}m"
            )
        if stats.max_abs_vz_m_s > args.max_hover_vz_m_s:
            raise ProbeFailure(
                f"hover max |vz| {stats.max_abs_vz_m_s:.3f}m/s exceeds "
                f"{args.max_hover_vz_m_s:.3f}m/s"
            )
        print(
            f"[hover] z={stats.median_z_m:.3f}m "
            f"span={stats.z_span_m:.3f}m max_vz={stats.max_abs_vz_m_s:.3f}m/s "
            f"max_tilt={stats.max_tilt_deg:.2f}deg "
            f"max_xy_drift={stats.max_xy_drift_m:.3f}m"
        )
        return stats.median_z_m

    def publish_vertical_velocity(
        up_m_s: float,
        ground_xyz: tuple[float, float, float],
    ) -> None:
        message = Twist()
        message.linear.z = float(up_m_s)
        deadline_s = time.monotonic() + args.vertical_command_duration
        period_s = 1.0 / args.command_rate_hz
        next_publish_s = 0.0
        while rclpy.ok() and time.monotonic() < deadline_s:
            now_s = time.monotonic()
            if now_s >= next_publish_s:
                node.velocity_pub.publish(message)
                next_publish_s = now_s + period_s
            rclpy.spin_once(node, timeout_sec=0.05)
            require_flight_safe(ground_xyz)

    def stop_and_measure_hold(
        ground_xyz: tuple[float, float, float],
    ) -> WindowStats:
        publish_for(node.velocity_pub, Twist(), args.zero_command_duration)
        samples = monitor_flight(args.post_command_hold, ground_xyz)
        recent_threshold_s = samples[-1].received_s - args.hold_window
        hold_samples = [
            sample for sample in samples if sample.received_s >= recent_threshold_s
        ]
        stats = window_stats(hold_samples, ground_xyz[:2])
        if stats.duration_s < 0.8 * args.hold_window:
            raise ProbeFailure("insufficient ground-truth coverage in hold window")
        if stats.z_span_m > args.max_hold_z_span_m:
            raise ProbeFailure(
                f"hold z span {stats.z_span_m:.3f}m exceeds "
                f"{args.max_hold_z_span_m:.3f}m"
            )
        if stats.max_abs_vz_m_s > args.max_hold_vz_m_s:
            raise ProbeFailure(
                f"hold max |vz| {stats.max_abs_vz_m_s:.3f}m/s exceeds "
                f"{args.max_hold_vz_m_s:.3f}m/s"
            )
        return stats

    def verify_vertical_response(
        label: str,
        up_m_s: float,
        starting_z_m: float,
        ground_xyz: tuple[float, float, float],
    ) -> float:
        print(
            f"[{label}] cmd_vel.z={up_m_s:+.2f}m/s for "
            f"{args.vertical_command_duration:g}s"
        )
        publish_vertical_velocity(up_m_s, ground_xyz)
        stats = stop_and_measure_hold(ground_xyz)
        delta_m = stats.median_z_m - starting_z_m
        directed_delta_m = delta_m if up_m_s > 0.0 else -delta_m
        if directed_delta_m < args.vertical_min_delta_m:
            raise ProbeFailure(
                f"{label} directed delta {directed_delta_m:.3f}m is below "
                f"{args.vertical_min_delta_m:.3f}m"
            )
        print(
            f"[{label}] delta={delta_m:+.3f}m hold_span={stats.z_span_m:.3f}m "
            f"max_xy_drift={stats.max_xy_drift_m:.3f}m"
        )
        return stats.median_z_m

    def landed_stably(
        ground_xyz: tuple[float, float, float],
        landing_started_s: float,
    ) -> bool:
        landing_samples = node.samples_since(landing_started_s)
        if not landing_samples:
            return False
        recent_threshold_s = (
            landing_samples[-1].received_s - args.land_stable_window
        )
        recent = [
            sample
            for sample in landing_samples
            if sample.received_s >= recent_threshold_s
        ]
        return landed_window_is_stable(
            recent,
            ground_xyz,
            now_s=time.monotonic(),
            minimum_duration_s=0.8 * args.land_stable_window,
            telemetry_max_age_s=args.telemetry_max_age,
            z_tolerance_m=args.land_z_tolerance_m,
            max_abs_vz_m_s=args.land_max_vz_m_s,
            max_xy_error_m=args.land_max_xy_error_m,
            max_tilt_deg=args.land_max_tilt_deg,
        )

    def cleanup(ground_xyz: tuple[float, float, float] | None) -> bool:
        cleanup_ok = True
        landing_verified = False
        print("[cleanup] sending zero velocity and land")
        try:
            publish_for(node.velocity_pub, Twist(), args.zero_command_duration)
            publish_for(node.land_pub, Bool(data=True), args.command_duration)
            landing_started_s = time.monotonic()
            if ground_xyz is not None:
                if not spin_until(
                    args.land_timeout,
                    lambda: landed_stably(ground_xyz, landing_started_s),
                ):
                    sample = node.latest
                    current_z = float("nan") if sample is None else sample.z_m
                    current_xy_error = (
                        float("nan")
                        if sample is None
                        else math.hypot(
                            sample.x_m - ground_xyz[0],
                            sample.y_m - ground_xyz[1],
                        )
                    )
                    current_tilt = (
                        float("nan") if sample is None else sample.tilt_deg
                    )
                    print(
                        f"FAIL: landing did not stabilize near ground; "
                        f"ground_z={ground_xyz[2]:.3f} current_z={current_z:.3f} "
                        f"xy_error={current_xy_error:.3f}m "
                        f"tilt={current_tilt:.2f}deg",
                        file=sys.stderr,
                    )
                    cleanup_ok = False
                else:
                    landing_verified = True
                    print("[cleanup] landed on settled ground")
        except Exception as exc:
            print(
                f"FAIL: cleanup landing command failed: {type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
            cleanup_ok = False

        disarm_publisher = node.arm_pub
        if landing_verified:
            print("[cleanup] requesting ground-gated force disarm")
            disarm_publisher = node.force_disarm_pub
        else:
            print("[cleanup] requesting normal safety-checked disarm")
        try:
            publish_for(
                disarm_publisher,
                Bool(data=landing_verified),
                args.command_duration,
            )
            if not spin_until(args.disarm_timeout, lambda: not node.armed):
                print("FAIL: timed out waiting for armed=false", file=sys.stderr)
                cleanup_ok = False
            else:
                print("[cleanup] armed=false")
        except Exception as exc:
            print(
                f"FAIL: cleanup disarm command failed: {type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
            cleanup_ok = False
        return cleanup_ok

    rclpy.init()
    node = MountainFlightProbe()
    ground_xyz: tuple[float, float, float] | None = None
    flight_ok = False
    failure: str | None = None
    try:
        print(
            f"[ready] waiting for /drone/heartbeat "
            f"({args.heartbeat_timeout:g}s timeout)"
        )
        if not spin_until(args.heartbeat_timeout, lambda: node.heartbeat is not None):
            raise ProbeFailure("timed out waiting for /drone/heartbeat")
        heartbeat = node.heartbeat or {}
        print(
            f"[ready] heartbeat mode={heartbeat.get('mode', 'UNKNOWN')} "
            f"armed={heartbeat.get('armed')}"
        )

        print(
            f"[ready] waiting for /sim/ground_truth/odom "
            f"({args.ground_truth_timeout:g}s timeout)"
        )
        if not spin_until(args.ground_truth_timeout, lambda: node.latest is not None):
            raise ProbeFailure("timed out waiting for /sim/ground_truth/odom")
        print(
            "[ready] waiting for ArduPilot LOCAL_POSITION_NED readiness "
            f"({args.local_position_timeout:g}s timeout)"
        )
        if not spin_until(
            args.local_position_timeout,
            lambda: node.local_position_ready,
        ):
            raise ProbeFailure(
                "timed out waiting for ArduPilot LOCAL_POSITION_NED readiness"
            )
        if not spin_until(args.endpoint_timeout, lambda: not node.missing_endpoints()):
            raise ProbeFailure(
                "missing ROS 2 endpoints: " + ", ".join(node.missing_endpoints())
            )
        print("[ready] command and telemetry endpoints are ready")

        if node.armed:
            raise ProbeFailure("vehicle is already armed; start from a clean stack")
        ground_xyz = settle_ground()
        arm()
        takeoff(ground_xyz)
        hover_z_m = verify_hover(ground_xyz)
        up_z_m = verify_vertical_response(
            "up", args.vertical_speed_m_s, hover_z_m, ground_xyz
        )
        down_z_m = verify_vertical_response(
            "down", -args.vertical_speed_m_s, up_z_m, ground_xyz
        )
        return_error_m = abs(down_z_m - hover_z_m)
        if return_error_m > args.return_tolerance_m:
            raise ProbeFailure(
                f"post-down altitude differs from pre-up hover by "
                f"{return_error_m:.3f}m; tolerance={args.return_tolerance_m:.3f}m"
            )
        print(
            f"[down] returned to pre-up hover within {return_error_m:.3f}m"
        )
        flight_ok = True
    except ProbeFailure as exc:
        failure = str(exc)
        print(f"FAIL: {failure}", file=sys.stderr)
    except KeyboardInterrupt:
        failure = "interrupted"
        print("FAIL: interrupted", file=sys.stderr)
    except Exception as exc:  # pragma: no cover - diagnostic for live runs.
        failure = f"{type(exc).__name__}: {exc}"
        print(f"FAIL: {failure}", file=sys.stderr)
    finally:
        cleanup_ok = cleanup(ground_xyz)
        node.destroy_node()
        rclpy.shutdown()

    if flight_ok and cleanup_ok:
        print(PASS_LINE)
        return 0
    if failure is None and not cleanup_ok:
        print("FAIL: flight phases passed but cleanup verification failed", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
