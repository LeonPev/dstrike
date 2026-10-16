"""Autonomous route controller for the optional dual-SITL target vehicle.

The pure controller in this module produces ROS ENU velocity and yaw-rate
requests only.  The existing MAVLink bridge remains the sole hardware adapter
and translates those requests into ArduPilot commands.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from pathlib import Path
import sys
import time
from typing import Protocol

# ROS 2 console scripts execute from the colcon install tree.  The simulation
# contracts keep repository-owned physics/config modules at /workspace/src.
WORKSPACE_ROOT = Path("/workspace")
if WORKSPACE_ROOT.is_dir() and str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from src.physics.scenario_config import (
    TwoDroneScenarioConfig,
    load_mountain_map_config,
    load_two_drone_config,
)
from src.physics.target_motion import TargetMotion, TargetMotionState
from src.physics.terrain_map import TerrainMap


HEARTBEAT_TOPIC = "drone/heartbeat"
ODOMETRY_TOPIC = "drone/odom"
ARM_TOPIC = "drone/arm"
TAKEOFF_TOPIC = "drone/takeoff_alt_m"
VELOCITY_TOPIC = "drone/cmd_vel"
ROUTE_STATE_TOPIC = "route/state"
FLIGHT_RELEASE_TOPIC = "/sim/target/flight_release"
CONTROL_PERIOD_S = 0.1


class TargetRouteState(str, Enum):
    WAIT_HEALTH = "WAIT_HEALTH"
    ARM = "ARM"
    STABILIZE = "STABILIZE"
    ROUTE = "ROUTE"
    ABORT = "ABORT"


class RouteReference(Protocol):
    def state_at(self, time_s: float) -> TargetMotionState: ...


class TerrainHeightProvider(Protocol):
    size_m: float

    def height_at(self, x_m: float, y_m: float) -> float: ...


@dataclass(frozen=True)
class TargetRouteControlConfig:
    target_agl_m: float
    geofence_radius_m: float
    min_clearance_m: float
    activation_takeoff_alt_m: float = 0.5
    post_release_takeoff_s: float = 0.75
    health_timeout_s: float = 45.0
    arm_timeout_s: float = 75.0
    stabilize_timeout_s: float = 20.0
    stabilize_duration_s: float = 8.0
    telemetry_stale_s: float = 1.0
    stabilize_xy_error_m: float = 0.25
    stabilize_speed_mps: float = 0.25
    stabilize_agl_error_m: float = 0.75
    stabilize_yaw_error_rad: float = math.radians(20.0)
    position_gain_s: float = 0.45
    agl_gain_s: float = 0.55
    yaw_gain_s: float = 0.6
    yaw_deadband_rad: float = math.radians(1.0)
    max_horizontal_speed_mps: float = 4.0
    max_vertical_speed_mps: float = 2.5
    max_yaw_rate_rad_s: float = math.radians(45.0)
    terrain_feedforward_step_s: float = 0.2

    def __post_init__(self) -> None:
        positive = {
            "target_agl_m": self.target_agl_m,
            "geofence_radius_m": self.geofence_radius_m,
            "min_clearance_m": self.min_clearance_m,
            "activation_takeoff_alt_m": self.activation_takeoff_alt_m,
            "post_release_takeoff_s": self.post_release_takeoff_s,
            "health_timeout_s": self.health_timeout_s,
            "arm_timeout_s": self.arm_timeout_s,
            "stabilize_timeout_s": self.stabilize_timeout_s,
            "stabilize_duration_s": self.stabilize_duration_s,
            "telemetry_stale_s": self.telemetry_stale_s,
            "stabilize_xy_error_m": self.stabilize_xy_error_m,
            "stabilize_speed_mps": self.stabilize_speed_mps,
            "stabilize_agl_error_m": self.stabilize_agl_error_m,
            "stabilize_yaw_error_rad": self.stabilize_yaw_error_rad,
            "position_gain_s": self.position_gain_s,
            "agl_gain_s": self.agl_gain_s,
            "yaw_gain_s": self.yaw_gain_s,
            "yaw_deadband_rad": self.yaw_deadband_rad,
            "max_horizontal_speed_mps": self.max_horizontal_speed_mps,
            "max_vertical_speed_mps": self.max_vertical_speed_mps,
            "max_yaw_rate_rad_s": self.max_yaw_rate_rad_s,
            "terrain_feedforward_step_s": self.terrain_feedforward_step_s,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.min_clearance_m > self.target_agl_m:
            raise ValueError("min_clearance_m cannot exceed target_agl_m")

    @classmethod
    def from_scenario(
        cls,
        scenario: TwoDroneScenarioConfig,
        terrain: TerrainHeightProvider,
    ) -> "TargetRouteControlConfig":
        target = scenario.target_drone
        safety = scenario.safety
        return cls(
            target_agl_m=target.altitude_agl_m,
            geofence_radius_m=(
                terrain.size_m / 2.0 - safety.geofence_margin_m
            ),
            min_clearance_m=safety.min_terrain_clearance_m,
            max_horizontal_speed_mps=max(4.0, 2.0 * target.speed_mps),
        )


@dataclass(frozen=True)
class TargetRouteObservation:
    now_s: float
    healthy: bool
    armed: bool
    position_xyz: tuple[float, float, float] | None
    velocity_xyz: tuple[float, float, float] | None
    yaw_rad: float | None
    heartbeat_age_s: float
    odometry_age_s: float


@dataclass(frozen=True)
class TargetRouteAction:
    state: TargetRouteState
    request_arm: bool = False
    request_takeoff_alt_m: float | None = None
    velocity_enu: tuple[float, float, float] | None = None
    yaw_rate_rad_s: float = 0.0
    release_flight: bool = False
    reference: TargetMotionState | None = None
    abort_reason: str | None = None


class TargetRouteController:
    """Pure guarded lifecycle and terrain-following route controller."""

    def __init__(
        self,
        config: TargetRouteControlConfig,
        reference: RouteReference,
        terrain: TerrainHeightProvider,
    ) -> None:
        self.config = config
        self.reference = reference
        self.terrain = terrain
        self.state = TargetRouteState.WAIT_HEALTH
        self.state_entered_s: float | None = None
        self.stable_since_s: float | None = None
        self.route_started_s: float | None = None
        self.flight_released = False
        self.abort_reason: str | None = None

    def tick(self, observation: TargetRouteObservation) -> TargetRouteAction:
        self._validate_observation_time(observation)
        if self.state_entered_s is None:
            self.state_entered_s = observation.now_s

        if self.state == TargetRouteState.ABORT:
            return self._zero_action()

        invalid_reason = self._invalid_telemetry_reason(observation)
        if invalid_reason is not None:
            return self._abort(invalid_reason, observation.now_s)

        if self.state == TargetRouteState.WAIT_HEALTH:
            if self._ready(observation):
                safety_reason = self._safety_reason(observation)
                if safety_reason is not None:
                    return self._abort(safety_reason, observation.now_s)
                self._transition(TargetRouteState.ARM, observation.now_s)
                return TargetRouteAction(self.state, request_arm=True)
            if self._state_elapsed(observation.now_s) >= self.config.health_timeout_s:
                return self._abort("health timeout", observation.now_s)
            return TargetRouteAction(self.state)

        stale_reason = self._stale_or_unhealthy_reason(observation)
        if stale_reason is not None:
            return self._abort(stale_reason, observation.now_s)
        safety_reason = self._safety_reason(observation)
        if safety_reason is not None:
            return self._abort(safety_reason, observation.now_s)

        if self.state == TargetRouteState.ARM:
            if observation.armed:
                self._transition(TargetRouteState.STABILIZE, observation.now_s)
                return self._stabilize_action()
            if self._state_elapsed(observation.now_s) >= self.config.arm_timeout_s:
                return self._abort("arm timeout", observation.now_s)
            return TargetRouteAction(self.state, request_arm=True)

        if self.state == TargetRouteState.STABILIZE:
            if not observation.armed:
                return self._abort("target disarmed during stabilization", observation.now_s)
            if self._stabilized(observation):
                if self.stable_since_s is None:
                    self.stable_since_s = observation.now_s
                if (
                    observation.now_s - self.stable_since_s
                    >= self.config.stabilize_duration_s
                ):
                    self._transition(TargetRouteState.ROUTE, observation.now_s)
                    self.route_started_s = observation.now_s
                    self.flight_released = True
                    return self._route_action(observation, release_flight=True)
            else:
                self.stable_since_s = None
            if (
                self._state_elapsed(observation.now_s)
                >= self.config.stabilize_timeout_s
            ):
                return self._abort("stabilization timeout", observation.now_s)
            return self._stabilize_action()

        if self.state == TargetRouteState.ROUTE:
            if not observation.armed:
                return self._abort("target disarmed during route", observation.now_s)
            return self._route_action(observation)

        raise RuntimeError(f"unsupported target route state {self.state}")

    def _route_action(
        self,
        observation: TargetRouteObservation,
        *,
        release_flight: bool = False,
    ) -> TargetRouteAction:
        if self.route_started_s is None:
            raise RuntimeError("ROUTE state requires a route start time")
        position = observation.position_xyz
        yaw_rad = observation.yaw_rad
        if position is None or yaw_rad is None:
            return self._abort("route telemetry missing", observation.now_s)

        elapsed_s = max(0.0, observation.now_s - self.route_started_s)
        desired = self.reference.state_at(elapsed_s)
        if elapsed_s < self.config.post_release_takeoff_s:
            return TargetRouteAction(
                state=self.state,
                release_flight=release_flight,
                reference=desired,
            )
        future = self.reference.state_at(
            elapsed_s + self.config.terrain_feedforward_step_s
        )

        raw_x = desired.velocity_xyz[0] + self.config.position_gain_s * (
            desired.position_xyz[0] - position[0]
        )
        raw_y = desired.velocity_xyz[1] + self.config.position_gain_s * (
            desired.position_xyz[1] - position[1]
        )
        velocity_x, velocity_y = _limit_horizontal(
            raw_x,
            raw_y,
            self.config.max_horizontal_speed_mps,
        )

        terrain_vertical_feedforward = (
            future.position_xyz[2] - desired.position_xyz[2]
        ) / self.config.terrain_feedforward_step_s
        current_agl_m = position[2] - self.terrain.height_at(
            position[0], position[1]
        )
        raw_z = (
            terrain_vertical_feedforward
            + self.config.position_gain_s
            * (desired.position_xyz[2] - position[2])
            + self.config.agl_gain_s
            * (self.config.target_agl_m - current_agl_m)
        )
        velocity_z = _clamp(
            raw_z,
            -self.config.max_vertical_speed_mps,
            self.config.max_vertical_speed_mps,
        )

        desired_yaw_rad = math.radians(desired.heading_deg)
        yaw_error_rad = _wrapped_angle(desired_yaw_rad - yaw_rad)
        yaw_rate = (
            0.0
            if abs(yaw_error_rad) <= self.config.yaw_deadband_rad
            else _clamp(
                self.config.yaw_gain_s * yaw_error_rad,
                -self.config.max_yaw_rate_rad_s,
                self.config.max_yaw_rate_rad_s,
            )
        )
        return TargetRouteAction(
            state=self.state,
            velocity_enu=(velocity_x, velocity_y, velocity_z),
            yaw_rate_rad_s=yaw_rate,
            release_flight=release_flight,
            reference=desired,
        )

    def _ready(self, observation: TargetRouteObservation) -> bool:
        return (
            observation.healthy
            and observation.position_xyz is not None
            and observation.velocity_xyz is not None
            and observation.yaw_rad is not None
            and observation.heartbeat_age_s <= self.config.telemetry_stale_s
            and observation.odometry_age_s <= self.config.telemetry_stale_s
        )

    def _stabilized(self, observation: TargetRouteObservation) -> bool:
        position = observation.position_xyz
        velocity = observation.velocity_xyz
        yaw_rad = observation.yaw_rad
        if position is None or velocity is None or yaw_rad is None:
            return False
        initial = self.reference.state_at(0.0)
        current_agl_m = position[2] - self.terrain.height_at(
            position[0], position[1]
        )
        yaw_error = abs(
            _wrapped_angle(math.radians(initial.heading_deg) - yaw_rad)
        )
        return (
            math.dist(position[:2], initial.position_xyz[:2])
            <= self.config.stabilize_xy_error_m
            and math.sqrt(sum(component * component for component in velocity))
            <= self.config.stabilize_speed_mps
            and abs(current_agl_m - self.config.target_agl_m)
            <= self.config.stabilize_agl_error_m
            and yaw_error <= self.config.stabilize_yaw_error_rad
        )

    def _invalid_telemetry_reason(
        self, observation: TargetRouteObservation
    ) -> str | None:
        for label, values in (
            ("position", observation.position_xyz),
            ("velocity", observation.velocity_xyz),
        ):
            if values is not None and not all(math.isfinite(value) for value in values):
                return f"non-finite {label} telemetry"
        if observation.yaw_rad is not None and not math.isfinite(observation.yaw_rad):
            return "non-finite yaw telemetry"
        return None

    def _stale_or_unhealthy_reason(
        self, observation: TargetRouteObservation
    ) -> str | None:
        if not observation.healthy:
            return "target health lost"
        if observation.heartbeat_age_s > self.config.telemetry_stale_s:
            return "heartbeat stale"
        if observation.odometry_age_s > self.config.telemetry_stale_s:
            return "odometry stale"
        if (
            observation.position_xyz is None
            or observation.velocity_xyz is None
            or observation.yaw_rad is None
        ):
            return "target telemetry missing"
        return None

    def _safety_reason(self, observation: TargetRouteObservation) -> str | None:
        position = observation.position_xyz
        if position is None:
            return None
        if (
            abs(position[0]) > self.config.geofence_radius_m
            or abs(position[1]) > self.config.geofence_radius_m
        ):
            return "target geofence violation"
        try:
            terrain_height_m = self.terrain.height_at(position[0], position[1])
        except ValueError:
            return "target is outside terrain bounds"
        if position[2] - terrain_height_m < self.config.min_clearance_m:
            return "unsafe target terrain clearance"
        return None

    def _abort(self, reason: str, now_s: float) -> TargetRouteAction:
        self.abort_reason = reason
        self._transition(TargetRouteState.ABORT, now_s)
        return self._zero_action()

    def _zero_action(self) -> TargetRouteAction:
        return TargetRouteAction(
            state=self.state,
            velocity_enu=(0.0, 0.0, 0.0),
            abort_reason=self.abort_reason,
        )

    def _stabilize_action(self) -> TargetRouteAction:
        """Let the bounded GUIDED takeoff own motor spool-up while held."""
        return TargetRouteAction(
            state=self.state,
            request_takeoff_alt_m=self.config.activation_takeoff_alt_m,
        )

    def _transition(self, state: TargetRouteState, now_s: float) -> None:
        self.state = state
        self.state_entered_s = now_s

    def _state_elapsed(self, now_s: float) -> float:
        if self.state_entered_s is None:
            return 0.0
        return max(0.0, now_s - self.state_entered_s)

    @staticmethod
    def _validate_observation_time(observation: TargetRouteObservation) -> None:
        if not math.isfinite(observation.now_s):
            raise ValueError("observation time must be finite")
        for name, age in (
            ("heartbeat_age_s", observation.heartbeat_age_s),
            ("odometry_age_s", observation.odometry_age_s),
        ):
            if math.isnan(age) or age < 0.0:
                raise ValueError(f"{name} must be non-negative")


def load_canonical_controller(root: str | Path) -> TargetRouteController:
    root_path = Path(root)
    map_assets = root_path / "models/scenarios/mountain_map"
    map_config = load_mountain_map_config(map_assets / "scenario.yaml")
    terrain = TerrainMap.from_config(map_config, map_assets)
    scenario = load_two_drone_config(
        root_path / "config/scenarios/mountain_two_drones.yaml",
        map_config_path=map_assets / "scenario.yaml",
        map_asset_dir=map_assets,
    )
    reference = TargetMotion(scenario.target_drone, terrain)
    return TargetRouteController(
        TargetRouteControlConfig.from_scenario(scenario, terrain),
        reference,
        terrain,
    )


def quaternion_yaw_rad(message: object) -> float:
    x = float(message.x)
    y = float(message.y)
    z = float(message.z)
    w = float(message.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("odometry quaternion must be finite and non-zero")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _limit_horizontal(x_mps: float, y_mps: float, limit_mps: float) -> tuple[float, float]:
    magnitude = math.hypot(x_mps, y_mps)
    if magnitude <= limit_mps:
        return x_mps, y_mps
    scale = limit_mps / magnitude
    return x_mps * scale, y_mps * scale


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(float(value), minimum), maximum)


def _wrapped_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def main(args: list[str] | None = None) -> None:  # pragma: no cover - live ROS adapter.
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32, String

    core = load_canonical_controller(Path("/workspace"))

    class TargetRouteControllerNode(Node):
        def __init__(self) -> None:
            super().__init__("target_route_controller")
            self.core = core
            self.healthy = False
            self.armed = False
            self.position_xyz: tuple[float, float, float] | None = None
            self.velocity_xyz: tuple[float, float, float] | None = None
            self.yaw_rad: float | None = None
            self.last_heartbeat_s: float | None = None
            self.last_odometry_s: float | None = None
            self.last_arm_publish_s = float("-inf")
            self.last_takeoff_publish_s = float("-inf")
            self.last_state: TargetRouteState | None = None
            self.flight_release_published = False

            latched_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.state_pub = self.create_publisher(
                String, ROUTE_STATE_TOPIC, latched_qos
            )
            self.release_pub = self.create_publisher(
                Bool, FLIGHT_RELEASE_TOPIC, latched_qos
            )
            self.arm_pub = self.create_publisher(Bool, ARM_TOPIC, 10)
            self.takeoff_pub = self.create_publisher(Float32, TAKEOFF_TOPIC, 10)
            self.velocity_pub = self.create_publisher(Twist, VELOCITY_TOPIC, 10)
            self.create_subscription(
                String, HEARTBEAT_TOPIC, self.on_heartbeat, 10
            )
            self.create_subscription(Odometry, ODOMETRY_TOPIC, self.on_odometry, 10)
            self.create_timer(CONTROL_PERIOD_S, self.tick)
            self.create_timer(1.0, self.publish_state)
            self.publish_state()

        def on_heartbeat(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
                status = str(payload.get("system_status", "")).upper()
                self.healthy = bool(payload.get("has_local_position_ned")) and status not in {
                    "MAV_STATE_CRITICAL",
                    "MAV_STATE_EMERGENCY",
                }
                self.armed = bool(payload.get("armed"))
                self.last_heartbeat_s = time.monotonic()
            except (TypeError, ValueError):
                self.healthy = False

        def on_odometry(self, message: Odometry) -> None:
            position = message.pose.pose.position
            velocity = message.twist.twist.linear
            values = (
                float(position.x),
                float(position.y),
                float(position.z),
                float(velocity.x),
                float(velocity.y),
                float(velocity.z),
            )
            try:
                yaw_rad = quaternion_yaw_rad(message.pose.pose.orientation)
            except ValueError:
                return
            if not all(math.isfinite(value) for value in values):
                return
            self.position_xyz = values[:3]
            self.velocity_xyz = values[3:]
            self.yaw_rad = yaw_rad
            self.last_odometry_s = time.monotonic()

        def observation(self, now_s: float) -> TargetRouteObservation:
            return TargetRouteObservation(
                now_s=now_s,
                healthy=self.healthy,
                armed=self.armed,
                position_xyz=self.position_xyz,
                velocity_xyz=self.velocity_xyz,
                yaw_rad=self.yaw_rad,
                heartbeat_age_s=(
                    math.inf
                    if self.last_heartbeat_s is None
                    else now_s - self.last_heartbeat_s
                ),
                odometry_age_s=(
                    math.inf
                    if self.last_odometry_s is None
                    else now_s - self.last_odometry_s
                ),
            )

        def publish_state(self) -> None:
            self.state_pub.publish(String(data=self.core.state.value))

        def tick(self) -> None:
            now_s = time.monotonic()
            action = self.core.tick(self.observation(now_s))
            if action.state != self.last_state:
                suffix = (
                    f": {action.abort_reason}" if action.abort_reason else ""
                )
                self.get_logger().info(
                    f"target route state -> {action.state.value}{suffix}"
                )
                self.last_state = action.state
                self.publish_state()

            if action.request_arm and now_s - self.last_arm_publish_s >= 1.0:
                self.arm_pub.publish(Bool(data=True))
                self.last_arm_publish_s = now_s

            if (
                action.request_takeoff_alt_m is not None
                and now_s - self.last_takeoff_publish_s >= 1.0
            ):
                self.takeoff_pub.publish(Float32(data=action.request_takeoff_alt_m))
                self.last_takeoff_publish_s = now_s

            if action.velocity_enu is not None:
                velocity = Twist()
                (
                    velocity.linear.x,
                    velocity.linear.y,
                    velocity.linear.z,
                ) = action.velocity_enu
                velocity.angular.z = action.yaw_rate_rad_s
                self.velocity_pub.publish(velocity)

            if action.release_flight and not self.flight_release_published:
                self.release_pub.publish(Bool(data=True))
                self.flight_release_published = True
                self.get_logger().info("released target flight after bounded stabilization")

    rclpy.init(args=args)
    node = TargetRouteControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
