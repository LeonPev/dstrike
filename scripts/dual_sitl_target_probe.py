#!/usr/bin/env python3
"""Bounded SCN-011 probe for actual dual-SITL target route flight."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.physics.scenario_config import load_mountain_map_config, load_two_drone_config
from src.physics.terrain_map import TerrainMap


ROUTE_STATE_TOPIC = "/target/route/state"
RELEASE_TOPIC = "/sim/target/flight_release"
TARGET_ODOM_TOPIC = "/sim/target/odom"
CHASER_ODOM_TOPIC = "/sim/chaser/ground_truth/odom"
TARGET_HEARTBEAT_TOPIC = "/target/drone/heartbeat"
TARGET_COMMAND_TOPIC = "/target/drone/cmd_vel"
ODOMETRY_STALE_S = 2.0
STRAIGHT_SPEED_TOLERANCE_MPS = 0.5
STRAIGHT_CROSS_TRACK_TOLERANCE_M = 0.5
STRAIGHT_AGL_TOLERANCE_M = 0.75
STRAIGHT_HEADING_TOLERANCE_DEG = 5.0


class ProbeFailure(RuntimeError):
    """Raised when an observable SCN-011 contract is violated."""


@dataclass(frozen=True)
class OdomSample:
    received_s: float
    position_xyz: tuple[float, float, float]
    velocity_xyz: tuple[float, float, float]
    yaw_rad: float


@dataclass
class EndpointReversalTracker:
    """Track the bounded dynamic turnaround separately from endpoint arrival."""

    route_length_m: float
    target_speed_mps: float
    endpoint_band_m: float = 3.0
    turnaround_timeout_s: float = 10.0
    speed_tolerance_mps: float = 0.5
    cross_track_tolerance_m: float = 0.5
    agl_tolerance_m: float = 0.75
    heading_tolerance_deg: float = 5.0
    entered_s: float | None = None
    max_along_m: float = float("-inf")

    def observe(
        self,
        *,
        sample_s: float,
        along_m: float,
        cross_track_m: float,
        reverse_velocity_error_mps: float,
        agl_error_m: float,
        reverse_heading_error_deg: float,
    ) -> bool:
        self.max_along_m = max(self.max_along_m, along_m)
        if along_m > self.route_length_m + self.endpoint_band_m:
            raise ProbeFailure(
                f"target overshot route endpoint: along={along_m:.3f}"
            )

        if self.entered_s is None:
            if along_m < self.route_length_m - self.endpoint_band_m:
                return False
            self.entered_s = sample_s

        elapsed_s = max(0.0, sample_s - self.entered_s)
        if elapsed_s > self.turnaround_timeout_s:
            raise ProbeFailure(
                "target did not settle after endpoint arrival within "
                f"{self.turnaround_timeout_s:.1f} s: along={along_m:.3f} "
                f"velocity_error={reverse_velocity_error_mps:.3f} "
                f"cross={cross_track_m:.3f} agl_error={agl_error_m:.3f} "
                f"heading_error={reverse_heading_error_deg:.3f}"
            )

        return (
            reverse_velocity_error_mps <= self.speed_tolerance_mps
            and abs(cross_track_m) <= self.cross_track_tolerance_m
            and agl_error_m <= self.agl_tolerance_m
            and reverse_heading_error_deg <= self.heading_tolerance_deg
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=300.0)
    parser.add_argument("--readiness-only", action="store_true")
    parser.add_argument("--readiness-observation-s", type=float, default=8.0)
    return parser.parse_args()


def yaw_from_odom(message: object) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error_rad(actual: float, desired: float) -> float:
    return math.atan2(math.sin(actual - desired), math.cos(actual - desired))


def route_coordinates(
    position_xyz: tuple[float, float, float],
    start_xy: tuple[float, float],
    unit_xy: tuple[float, float],
) -> tuple[float, float]:
    dx = position_xyz[0] - start_xy[0]
    dy = position_xyz[1] - start_xy[1]
    along = dx * unit_xy[0] + dy * unit_xy[1]
    cross = -dx * unit_xy[1] + dy * unit_xy[0]
    return along, cross


def horizontal_velocity_error_mps(
    sample: OdomSample,
    unit_xy: tuple[float, float],
    speed_mps: float,
    direction: int,
) -> float:
    desired_x = direction * speed_mps * unit_xy[0]
    desired_y = direction * speed_mps * unit_xy[1]
    return math.hypot(
        sample.velocity_xyz[0] - desired_x,
        sample.velocity_xyz[1] - desired_y,
    )


def require_fresh_odom(sample: OdomSample) -> None:
    age_s = time.monotonic() - sample.received_s
    if age_s > ODOMETRY_STALE_S:
        raise ProbeFailure(f"target physics odometry stale: age={age_s:.3f} s")


def odom_sample(message: object) -> OdomSample:
    position = message.pose.pose.position
    velocity = message.twist.twist.linear
    yaw_rad = yaw_from_odom(message)
    values = (
        float(position.x),
        float(position.y),
        float(position.z),
        float(velocity.x),
        float(velocity.y),
        float(velocity.z),
    )
    if not all(math.isfinite(value) for value in (*values, yaw_rad)):
        raise ProbeFailure("odometry contains non-finite position, velocity, or yaw")
    return OdomSample(
        received_s=time.monotonic(),
        position_xyz=values[:3],
        velocity_xyz=values[3:],
        yaw_rad=yaw_rad,
    )


def run(args: argparse.Namespace) -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    root = ROOT
    config = load_two_drone_config(root / "config/scenarios/mountain_two_drones.yaml")
    map_dir = root / "models/scenarios/mountain_map"
    map_config = load_mountain_map_config(map_dir / "scenario.yaml")
    terrain = TerrainMap.from_config(map_config, map_dir)

    target_cfg = config.target_drone
    safety = config.safety
    yaw_rad = math.radians(target_cfg.normalized_heading_deg)
    unit_xy = (math.cos(yaw_rad), math.sin(yaw_rad))
    reverse_yaw_rad = math.atan2(-unit_xy[1], -unit_xy[0])
    usable_radius = terrain.size_m / 2.0 - safety.geofence_margin_m

    state_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    rclpy.init()
    node = rclpy.create_node("scn011_dual_sitl_target_probe")
    target_samples: list[OdomSample] = []
    chaser_samples: list[OdomSample] = []
    heartbeat: dict[str, object] | None = None
    heartbeat_received_s = 0.0
    route_state = ""
    route_state_received_s = 0.0
    state_history: list[str] = []
    release_received_s: float | None = None
    release_count = 0
    command_count = 0
    nonzero_command_count = 0
    callback_error: Exception | None = None

    def on_target_odom(message: Odometry) -> None:
        nonlocal callback_error
        try:
            target_samples.append(odom_sample(message))
        except Exception as exc:  # surfaced from the main spin loop
            callback_error = exc

    def on_chaser_odom(message: Odometry) -> None:
        nonlocal callback_error
        try:
            chaser_samples.append(odom_sample(message))
        except Exception as exc:  # surfaced from the main spin loop
            callback_error = exc

    def on_heartbeat(message: String) -> None:
        nonlocal heartbeat, heartbeat_received_s
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict):
            heartbeat = value
            heartbeat_received_s = time.monotonic()

    def on_state(message: String) -> None:
        nonlocal route_state, route_state_received_s
        route_state = message.data
        route_state_received_s = time.monotonic()
        if not state_history or state_history[-1] != route_state:
            state_history.append(route_state)
            print(f"target-route-state={route_state}", flush=True)

    def on_release(message: Bool) -> None:
        nonlocal release_received_s, release_count
        if message.data:
            release_count += 1
            if release_received_s is None:
                release_received_s = time.monotonic()

    def on_command(message: Twist) -> None:
        nonlocal command_count, nonzero_command_count
        command_count += 1
        magnitude = math.sqrt(
            message.linear.x**2 + message.linear.y**2 + message.linear.z**2
        )
        if magnitude > 0.1:
            nonzero_command_count += 1

    subscriptions = [
        node.create_subscription(Odometry, TARGET_ODOM_TOPIC, on_target_odom, 10),
        node.create_subscription(Odometry, CHASER_ODOM_TOPIC, on_chaser_odom, 10),
        node.create_subscription(String, TARGET_HEARTBEAT_TOPIC, on_heartbeat, 10),
        node.create_subscription(String, ROUTE_STATE_TOPIC, on_state, state_qos),
        node.create_subscription(Bool, RELEASE_TOPIC, on_release, state_qos),
        node.create_subscription(Twist, TARGET_COMMAND_TOPIC, on_command, 10),
    ]

    overall_deadline = time.monotonic() + args.timeout_s
    route_observed_s: float | None = None
    route_sample_index = 0
    try:
        while time.monotonic() < overall_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if callback_error is not None:
                raise ProbeFailure(str(callback_error))
            if route_state == "ABORT":
                raise ProbeFailure("target route controller entered ABORT")
            ready_heartbeat = (
                heartbeat is not None
                and int(heartbeat.get("sysid", 0)) == 2
                and bool(heartbeat.get("armed"))
                and str(heartbeat.get("mode", "")).upper() == "GUIDED"
            )
            if (
                route_state == "ROUTE"
                and release_received_s is not None
                and ready_heartbeat
                and target_samples
                and chaser_samples
            ):
                route_observed_s = time.monotonic()
                route_sample_index = len(target_samples) - 1
                break
        if route_observed_s is None:
            raise ProbeFailure(
                "timed out waiting for target ROUTE, release, GUIDED/armed heartbeat, and odometry"
            )

        pre_release = [
            sample
            for sample in target_samples
            if release_received_s is not None and sample.received_s < release_received_s
        ]
        if not pre_release:
            raise ProbeFailure("no pre-release target ground-truth samples were observed")
        expected_start_z = (
            terrain.height_at(*target_cfg.start_xy_m) + target_cfg.altitude_agl_m
        )
        max_pre_xy_error = max(
            math.dist(sample.position_xyz[:2], target_cfg.start_xy_m)
            for sample in pre_release
        )
        max_pre_z_error = max(
            abs(sample.position_xyz[2] - expected_start_z) for sample in pre_release
        )
        max_pre_speed = max(
            math.sqrt(sum(component * component for component in sample.velocity_xyz))
            for sample in pre_release
        )
        if max_pre_xy_error > 0.25 or max_pre_z_error > 0.75 or max_pre_speed > 0.25:
            raise ProbeFailure(
                "pre-release sky hold moved "
                f"xy_error={max_pre_xy_error:.3f} z_error={max_pre_z_error:.3f} "
                f"speed={max_pre_speed:.3f}"
            )

        first_route_sample = target_samples[route_sample_index]
        readiness_deadline = route_observed_s + args.readiness_observation_s
        while time.monotonic() < min(readiness_deadline, overall_deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
            if callback_error is not None:
                raise ProbeFailure(str(callback_error))
            if route_state == "ABORT":
                raise ProbeFailure("target route controller entered ABORT")

        require_fresh_odom(target_samples[-1])
        readiness_samples = target_samples[route_sample_index:]
        if len(readiness_samples) < 20:
            raise ProbeFailure(
                f"insufficient route odometry samples: {len(readiness_samples)}"
            )
        readiness_last = readiness_samples[-1]
        along_start, _ = route_coordinates(
            first_route_sample.position_xyz, target_cfg.start_xy_m, unit_xy
        )
        along_last, _ = route_coordinates(
            readiness_last.position_xyz, target_cfg.start_xy_m, unit_xy
        )
        readiness_displacement = along_last - along_start
        required_displacement = max(
            3.0, target_cfg.speed_mps * args.readiness_observation_s * 0.35
        )
        if readiness_displacement < required_displacement:
            raise ProbeFailure(
                "target did not advance along the first leg: "
                f"displacement={readiness_displacement:.3f} required={required_displacement:.3f}"
            )
        if nonzero_command_count < 10:
            raise ProbeFailure(
                f"too few nonzero target velocity commands: {nonzero_command_count}"
            )

        if args.readiness_only:
            samples_to_validate = readiness_samples
            straight_phases = [
                (
                    "outbound-readiness",
                    1,
                    [
                        sample
                        for sample in readiness_samples
                        if sample.received_s >= readiness_last.received_s - 2.0
                    ],
                )
            ]
            turnaround_duration_s: float | None = None
            turnaround_max_along_m: float | None = None
        else:
            reversal = EndpointReversalTracker(
                route_length_m=target_cfg.route_length_m,
                target_speed_mps=target_cfg.speed_mps,
            )
            last_turnaround_sample_s = float("-inf")
            while time.monotonic() < overall_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if callback_error is not None:
                    raise ProbeFailure(str(callback_error))
                if route_state == "ABORT":
                    raise ProbeFailure("target route controller entered ABORT")
                sample = target_samples[-1]
                require_fresh_odom(sample)
                if sample.received_s <= last_turnaround_sample_s:
                    continue
                last_turnaround_sample_s = sample.received_s
                along, cross = route_coordinates(
                    sample.position_xyz, target_cfg.start_xy_m, unit_xy
                )
                agl_error = abs(
                    sample.position_xyz[2]
                    - terrain.height_at(*sample.position_xyz[:2])
                    - target_cfg.altitude_agl_m
                )
                reverse_heading_error_deg = math.degrees(
                    abs(angle_error_rad(sample.yaw_rad, reverse_yaw_rad))
                )
                if reversal.observe(
                    sample_s=sample.received_s,
                    along_m=along,
                    cross_track_m=cross,
                    reverse_velocity_error_mps=horizontal_velocity_error_mps(
                        sample,
                        unit_xy,
                        target_cfg.speed_mps,
                        -1,
                    ),
                    agl_error_m=agl_error,
                    reverse_heading_error_deg=reverse_heading_error_deg,
                ):
                    turnaround_settled_s = time.monotonic()
                    turnaround_settled_sample_s = sample.received_s
                    turnaround_settled_along_m = along
                    break
            else:
                raise ProbeFailure(
                    "target did not reach and complete the endpoint reversal; "
                    f"max_along={reversal.max_along_m:.3f}"
                )

            if reversal.entered_s is None:
                raise ProbeFailure("endpoint reversal settled without endpoint arrival")
            turnaround_duration_s = (
                turnaround_settled_sample_s - reversal.entered_s
            )
            turnaround_max_along_m = reversal.max_along_m
            resume_deadline = min(turnaround_settled_s + 10.0, overall_deadline)
            while time.monotonic() < resume_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if callback_error is not None:
                    raise ProbeFailure(str(callback_error))
                if route_state == "ABORT":
                    raise ProbeFailure("target route controller entered ABORT")
                sample = target_samples[-1]
                require_fresh_odom(sample)
                along, _ = route_coordinates(
                    sample.position_xyz, target_cfg.start_xy_m, unit_xy
                )
                if turnaround_settled_along_m - along >= 5.0:
                    break
            else:
                raise ProbeFailure("target did not resume the opposite straight leg")
            samples_to_validate = target_samples[route_sample_index:]
            straight_phases = [
                (
                    "outbound",
                    1,
                    [
                        sample
                        for sample in samples_to_validate
                        if readiness_last.received_s
                        <= sample.received_s
                        < reversal.entered_s
                    ],
                ),
                (
                    "return",
                    -1,
                    [
                        sample
                        for sample in samples_to_validate
                        if sample.received_s >= turnaround_settled_sample_s
                    ],
                ),
            ]

        min_separation = math.inf
        max_cross_track = 0.0
        max_agl_error = 0.0
        min_clearance = math.inf
        for sample in samples_to_validate:
            along, cross = route_coordinates(
                sample.position_xyz, target_cfg.start_xy_m, unit_xy
            )
            del along
            max_cross_track = max(max_cross_track, abs(cross))
            terrain_height = terrain.height_at(*sample.position_xyz[:2])
            clearance = sample.position_xyz[2] - terrain_height
            min_clearance = min(min_clearance, clearance)
            max_agl_error = max(
                max_agl_error, abs(clearance - target_cfg.altitude_agl_m)
            )
            if (
                abs(sample.position_xyz[0]) > usable_radius
                or abs(sample.position_xyz[1]) > usable_radius
            ):
                raise ProbeFailure("target violated the configured geofence")
            if clearance < safety.min_terrain_clearance_m:
                raise ProbeFailure(
                    f"target terrain clearance unsafe: {clearance:.3f} m"
                )
            if chaser_samples:
                nearest_chaser = min(
                    chaser_samples,
                    key=lambda candidate: abs(candidate.received_s - sample.received_s),
                )
                min_separation = min(
                    min_separation,
                    math.dist(sample.position_xyz, nearest_chaser.position_xyz),
                )

        max_straight_cross_track = 0.0
        max_straight_agl_error = 0.0
        max_straight_speed_error = 0.0
        max_straight_heading_error_deg = 0.0
        for phase_name, direction, phase_samples in straight_phases:
            if len(phase_samples) < 10:
                raise ProbeFailure(
                    f"insufficient {phase_name} straight-leg samples: {len(phase_samples)}"
                )
            desired_yaw = yaw_rad if direction > 0 else reverse_yaw_rad
            for sample in phase_samples:
                _, cross = route_coordinates(
                    sample.position_xyz,
                    target_cfg.start_xy_m,
                    unit_xy,
                )
                clearance = sample.position_xyz[2] - terrain.height_at(
                    *sample.position_xyz[:2]
                )
                max_straight_cross_track = max(
                    max_straight_cross_track,
                    abs(cross),
                )
                max_straight_agl_error = max(
                    max_straight_agl_error,
                    abs(clearance - target_cfg.altitude_agl_m),
                )
                max_straight_speed_error = max(
                    max_straight_speed_error,
                    horizontal_velocity_error_mps(
                        sample,
                        unit_xy,
                        target_cfg.speed_mps,
                        direction,
                    ),
                )
                max_straight_heading_error_deg = max(
                    max_straight_heading_error_deg,
                    math.degrees(abs(angle_error_rad(sample.yaw_rad, desired_yaw))),
                )
        if max_cross_track > 1.5:
            raise ProbeFailure(f"target cross-track error too high: {max_cross_track:.3f} m")
        if max_agl_error > 3.0:
            raise ProbeFailure(f"target AGL error too high: {max_agl_error:.3f} m")
        if max_straight_cross_track > STRAIGHT_CROSS_TRACK_TOLERANCE_M:
            raise ProbeFailure(
                "straight-leg cross-track error too high: "
                f"{max_straight_cross_track:.3f} m"
            )
        if max_straight_agl_error > STRAIGHT_AGL_TOLERANCE_M:
            raise ProbeFailure(
                f"straight-leg AGL error too high: {max_straight_agl_error:.3f} m"
            )
        if max_straight_speed_error > STRAIGHT_SPEED_TOLERANCE_MPS:
            raise ProbeFailure(
                "straight-leg horizontal velocity error too high: "
                f"{max_straight_speed_error:.3f} m/s"
            )
        if max_straight_heading_error_deg > STRAIGHT_HEADING_TOLERANCE_DEG:
            raise ProbeFailure(
                "straight-leg heading error too high: "
                f"{max_straight_heading_error_deg:.3f} deg"
            )
        if min_separation < safety.min_drone_separation_m:
            raise ProbeFailure(
                f"minimum drone separation violated: {min_separation:.3f} m"
            )
        if heartbeat is None or time.monotonic() - heartbeat_received_s > 2.0:
            raise ProbeFailure("target heartbeat became stale")
        if not route_state_received_s or time.monotonic() - route_state_received_s > 2.0:
            raise ProbeFailure("target route state became stale")
        if release_count != 1:
            raise ProbeFailure(f"expected exactly one target release, observed {release_count}")

        print(f"target-state-history={','.join(state_history)}")
        print(
            f"pre-release-xy-error-max-m={max_pre_xy_error:.3f} "
            f"z-error-max-m={max_pre_z_error:.3f} speed-max-mps={max_pre_speed:.3f}"
        )
        print(
            f"target-first-leg-displacement-m={readiness_displacement:.3f} "
            f"commands={command_count} nonzero_commands={nonzero_command_count}"
        )
        print(
            f"target-cross-track-max-m={max_cross_track:.3f} "
            f"agl-error-max-m={max_agl_error:.3f} "
            f"clearance-min-m={min_clearance:.3f} "
            f"separation-min-m={min_separation:.3f} releases={release_count}"
        )
        print(
            f"straight-cross-track-max-m={max_straight_cross_track:.3f} "
            f"straight-agl-error-max-m={max_straight_agl_error:.3f} "
            f"straight-velocity-error-max-mps={max_straight_speed_error:.3f} "
            f"straight-heading-error-max-deg={max_straight_heading_error_deg:.3f}"
        )
        if turnaround_duration_s is not None and turnaround_max_along_m is not None:
            print(
                f"turnaround-duration-s={turnaround_duration_s:.3f} "
                f"turnaround-max-along-m={turnaround_max_along_m:.3f}"
            )
    finally:
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    args = parse_args()
    if args.timeout_s <= 0.0 or args.readiness_observation_s <= 0.0:
        print("ERROR: timeouts must be positive", file=sys.stderr)
        return 2
    try:
        run(args)
    except Exception as exc:
        print(f"ERROR: SCN-011 target probe failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
