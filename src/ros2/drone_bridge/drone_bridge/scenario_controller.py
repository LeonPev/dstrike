"""Two-drone chaser mission state machine and ROS 2 adapter."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml

from .formation_guidance import formation_command


class MissionState(str, Enum):
    IDLE = "IDLE"
    HEALTH_CHECK = "HEALTH_CHECK"
    ARM = "ARM"
    TAKEOFF = "TAKEOFF"
    INTERCEPT = "INTERCEPT"
    FORMATION_HOLD = "FORMATION_HOLD"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


@dataclass(frozen=True)
class ControllerConfig:
    takeoff_agl_m: float
    formation_offset_m: tuple[float, float, float]
    mission_timeout_s: float
    formation_threshold_m: float
    formation_hold_s: float
    min_clearance_m: float
    min_separation_m: float
    max_speed_mps: float
    geofence_radius_m: float
    state_timeout_s: float = 60.0


@dataclass(frozen=True)
class Observation:
    now_s: float
    healthy: bool
    armed: bool
    chaser_xyz: tuple[float, float, float] | None
    target_xyz: tuple[float, float, float] | None
    terrain_height_m: float | None
    target_yaw_rad: float = 0.0
    target_velocity_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Action:
    state: MissionState
    arm: bool = False
    takeoff_alt_m: float | None = None
    velocity_xyz: tuple[float, float, float] | None = None
    formation_error_m: float | None = None
    abort_reason: str | None = None


class MissionController:
    """Deterministic state machine independent of ROS and wall-clock time."""

    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self.state = MissionState.IDLE
        self.state_entered_s = 0.0
        self.mission_started_s: float | None = None
        self.spawn_z_m: float | None = None
        self.spawn_clearance_m: float | None = None
        self.clearance_enforced = False
        self.hold_started_s: float | None = None
        self.abort_reason: str | None = None

    def start(self, now_s: float) -> None:
        if self.state == MissionState.IDLE:
            self.mission_started_s = now_s
            self._transition(MissionState.HEALTH_CHECK, now_s)

    def tick(self, observation: Observation) -> Action:
        now = observation.now_s
        if self.state in (MissionState.IDLE, MissionState.ABORT):
            return Action(self.state, abort_reason=self.abort_reason)
        if self.mission_started_s is not None and now - self.mission_started_s > self.config.mission_timeout_s:
            return self._abort("mission timeout", now)
        if now - self.state_entered_s > self.config.state_timeout_s and self.state not in (
            MissionState.INTERCEPT, MissionState.FORMATION_HOLD, MissionState.COMPLETE
        ):
            return self._abort(f"{self.state.value} timeout", now)

        chaser = observation.chaser_xyz
        target = observation.target_xyz
        if chaser is not None:
            if abs(chaser[0]) > self.config.geofence_radius_m or abs(chaser[1]) > self.config.geofence_radius_m:
                return self._abort("geofence violation", now)
            if self.spawn_z_m is None:
                self.spawn_z_m = chaser[2]
        clearance = None
        if chaser is not None and observation.terrain_height_m is not None:
            clearance = chaser[2] - observation.terrain_height_m
            if self.spawn_clearance_m is None:
                self.spawn_clearance_m = clearance
            if clearance >= self.config.min_clearance_m:
                self.clearance_enforced = True
            if self.clearance_enforced and clearance < self.config.min_clearance_m:
                return self._abort("unsafe terrain clearance", now)
        if chaser is not None and target is not None:
            separation = math.dist(chaser, target)
            if separation < self.config.min_separation_m:
                return self._abort("minimum separation violation", now)

        if self.state == MissionState.HEALTH_CHECK:
            if observation.healthy and chaser is not None and target is not None:
                self._transition(MissionState.ARM, now)
            return Action(self.state)
        if self.state == MissionState.ARM:
            if observation.armed:
                self._transition(MissionState.TAKEOFF, now)
                return Action(self.state, takeoff_alt_m=self.config.takeoff_agl_m)
            return Action(self.state, arm=True)
        if self.state == MissionState.TAKEOFF:
            if chaser is not None and self.spawn_z_m is not None:
                required_gain = self.config.takeoff_agl_m - max(0.0, self.spawn_clearance_m or 0.0)
                if chaser[2] - self.spawn_z_m >= max(0.0, required_gain - 1.0):
                    if clearance is None or clearance < self.config.min_clearance_m:
                        return self._abort("cannot enter INTERCEPT below minimum clearance", now)
                    self.clearance_enforced = True
                    self._transition(MissionState.INTERCEPT, now)
            return Action(self.state)

        error, velocity = self._guidance(
            chaser, target, observation.target_yaw_rad,
            observation.target_velocity_xyz,
        )
        if error is not None and error <= self.config.formation_threshold_m:
            if self.state == MissionState.INTERCEPT:
                self.hold_started_s = now
                self._transition(MissionState.FORMATION_HOLD, now)
            elif self.state == MissionState.FORMATION_HOLD and self.hold_started_s is not None:
                if now - self.hold_started_s >= self.config.formation_hold_s:
                    self._transition(MissionState.COMPLETE, now)
        elif self.state == MissionState.FORMATION_HOLD:
            self.hold_started_s = None
            self._transition(MissionState.INTERCEPT, now)
        return Action(self.state, velocity_xyz=velocity, formation_error_m=error)

    def _guidance(self, chaser, target, target_yaw_rad=0.0, target_velocity_xyz=(0.0, 0.0, 0.0)):
        if chaser is None or target is None:
            return None, None
        result = formation_command(
            chaser, target, target_yaw_rad, self.config.formation_offset_m,
            self.config.max_speed_mps,
            target_velocity_xyz=target_velocity_xyz,
        )
        return result.error_m, result.velocity_xyz

    def _transition(self, state: MissionState, now_s: float) -> None:
        self.state = state
        self.state_entered_s = now_s

    def _abort(self, reason: str, now_s: float) -> Action:
        self.abort_reason = reason
        self._transition(MissionState.ABORT, now_s)
        return Action(self.state, velocity_xyz=(0.0, 0.0, 0.0), abort_reason=reason)


def _load_configuration(root: Path) -> tuple[ControllerConfig, np.ndarray, float]:
    scenario = yaml.safe_load((root / "config/scenarios/mountain_two_drones.yaml").read_text())
    map_config = yaml.safe_load((root / "models/scenarios/mountain_map/scenario.yaml").read_text())
    safety = scenario["safety"]
    chaser = scenario["chaser_drone"]
    size_m = float(map_config["terrain"]["size_m"])
    config = ControllerConfig(
        takeoff_agl_m=float(chaser["takeoff_agl_m"]),
        formation_offset_m=tuple(map(float, chaser["formation_offset_target_body_m"])),
        mission_timeout_s=float(safety["mission_timeout_s"]),
        formation_threshold_m=float(safety["formation_error_threshold_m"]),
        formation_hold_s=float(safety["formation_hold_required_s"]),
        min_clearance_m=float(safety["min_terrain_clearance_m"]),
        min_separation_m=float(safety["min_drone_separation_m"]),
        max_speed_mps=float(safety["max_chaser_speed_mps"]),
        geofence_radius_m=size_m / 2.0 - float(safety["geofence_margin_m"]),
    )
    heights = np.load(root / "models/scenarios/mountain_map/heightfield_m.npy")
    return config, heights, size_m


def _terrain_height(heights: np.ndarray, size_m: float, x: float, y: float) -> float:
    radius = size_m / 2.0
    row = (radius - y) * (heights.shape[0] - 1) / size_m
    col = (x + radius) * (heights.shape[1] - 1) / size_m
    r0 = min(max(int(math.floor(row)), 0), heights.shape[0] - 2)
    c0 = min(max(int(math.floor(col)), 0), heights.shape[1] - 2)
    ty, tx = row - r0, col - c0
    h00, h10 = float(heights[r0, c0]), float(heights[r0, c0 + 1])
    h01, h11 = float(heights[r0 + 1, c0]), float(heights[r0 + 1, c0 + 1])
    if tx + ty <= 1.0:
        return h00 + tx * (h10 - h00) + ty * (h01 - h00)
    return h11 + (1.0 - tx) * (h01 - h11) + (1.0 - ty) * (h10 - h11)


def main() -> None:  # pragma: no cover - exercised in Docker live check.
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32, String

    root = Path("/workspace")
    config, heights, size_m = _load_configuration(root)

    class ScenarioControllerNode(Node):
        def __init__(self) -> None:
            super().__init__("scenario_controller")
            self.core = MissionController(config)
            self.chaser = None
            self.target = None
            self.target_yaw_rad = 0.0
            self.target_velocity = (0.0, 0.0, 0.0)
            self.healthy = False
            self.armed = False
            self.last_state = None
            self.last_arm_request_s = float("-inf")
            state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.state_pub = self.create_publisher(String, "/scenario/state", state_qos)
            self.error_pub = self.create_publisher(Float32, "/scenario/formation_error_m", 10)
            self.arm_pub = self.create_publisher(Bool, "/drone/arm", 10)
            self.takeoff_pub = self.create_publisher(Float32, "/drone/takeoff_alt_m", 10)
            self.velocity_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
            self.create_subscription(Bool, "/scenario/start", self.on_start, 10)
            self.create_subscription(Odometry, "/drone/odom", lambda m: self.on_odom(m, True), 10)
            self.create_subscription(Odometry, "/sim/target/odom", lambda m: self.on_odom(m, False), 10)
            self.create_subscription(String, "/drone/heartbeat", self.on_heartbeat, 10)
            self.create_timer(0.1, self.tick)
            self.create_timer(1.0, self.publish_state)
            self.publish_state()

        def on_start(self, message):
            if message.data:
                self.core.start(time.monotonic())
                self.publish_state()

        def on_odom(self, message, chaser):
            p = message.pose.pose.position
            value = (float(p.x), float(p.y), float(p.z))
            if all(math.isfinite(v) for v in value):
                if chaser:
                    self.chaser = value
                else:
                    self.target = value
                    q = message.pose.pose.orientation
                    self.target_yaw_rad = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                    )
                    v = message.twist.twist.linear
                    velocity = (float(v.x), float(v.y), float(v.z))
                    if all(math.isfinite(component) for component in velocity):
                        self.target_velocity = velocity

        def on_heartbeat(self, message):
            try:
                data = json.loads(message.data)
                status = str(data.get("system_status", "")).upper()
                self.healthy = bool(data.get("has_local_position_ned")) and status not in ("MAV_STATE_CRITICAL", "MAV_STATE_EMERGENCY")
                self.armed = bool(data.get("armed"))
            except (ValueError, TypeError):
                self.healthy = False

        def publish_state(self):
            self.state_pub.publish(String(data=self.core.state.value))
            self.last_state = self.core.state

        def tick(self):
            terrain = _terrain_height(heights, size_m, *self.chaser[:2]) if self.chaser else None
            action = self.core.tick(Observation(
                time.monotonic(), self.healthy, self.armed, self.chaser,
                self.target, terrain, self.target_yaw_rad, self.target_velocity,
            ))
            if action.state != self.last_state:
                self.get_logger().info(f"scenario state -> {action.state.value}" + (f": {action.abort_reason}" if action.abort_reason else ""))
                self.publish_state()
            if action.arm:
                now_s = time.monotonic()
                if now_s - self.last_arm_request_s >= 1.0:
                    self.arm_pub.publish(Bool(data=True))
                    self.last_arm_request_s = now_s
            if action.takeoff_alt_m is not None:
                self.takeoff_pub.publish(Float32(data=action.takeoff_alt_m))
            if action.formation_error_m is not None:
                self.error_pub.publish(Float32(data=action.formation_error_m))
            if action.velocity_xyz is not None:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.linear.z = action.velocity_xyz
                self.velocity_pub.publish(msg)

    rclpy.init()
    node = ScenarioControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
