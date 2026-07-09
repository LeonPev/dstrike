"""ROS 2 node that bridges ArduPilot MAVLink telemetry and commands."""

from __future__ import annotations

from dataclasses import dataclass
import json
import queue
import threading
import time
from typing import Any

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from drone_bridge.frame_conventions import (
    GlobalPositionInt,
    GlobalPositionOrigin,
    LocalPositionNed,
    attitude_rpy_to_quaternion_wxyz,
    global_position_int_to_local_ned,
    local_position_ned_to_ros_odom_fields,
    ros_enu_twist_to_mavlink_ned_velocity,
    ros_yaw_rate_to_mavlink_ned,
)


LOCAL_NED_VELOCITY_TYPE_MASK = 1479


@dataclass(frozen=True)
class ArmCommand:
    arm: bool


@dataclass(frozen=True)
class TakeoffCommand:
    altitude_m: float


@dataclass(frozen=True)
class VelocityCommand:
    north_m_s: float
    east_m_s: float
    down_m_s: float
    yaw_rate_rad_s: float


@dataclass(frozen=True)
class MavlinkSnapshot:
    heartbeat: dict[str, Any] | None
    local_position: LocalPositionNed | None
    attitude_quaternion_wxyz: tuple[float, float, float, float] | None
    error: str | None


class MavlinkWorker:
    """Owns the blocking pymavlink connection on a background thread."""

    def __init__(self, endpoint: str, logger: Any) -> None:
        self._endpoint = endpoint
        self._logger = logger
        self._commands: queue.Queue[ArmCommand | TakeoffCommand | VelocityCommand] = (
            queue.Queue()
        )
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mavlink-worker")
        self._thread.daemon = True
        self._connection: Any | None = None
        self._mavutil: Any | None = None
        self._target_system = 0
        self._target_component = 0
        self._last_guided_request_s = 0.0
        self._seen_message_types: set[str] = set()
        self._heartbeat: dict[str, Any] | None = None
        self._local_position: LocalPositionNed | None = None
        self._global_origin: GlobalPositionOrigin | None = None
        self._attitude_quaternion_wxyz: tuple[float, float, float, float] | None = None
        self._error: str | None = None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        if self._connection is not None:
            self._connection.close()

    def enqueue_arm(self, arm: bool) -> None:
        self._commands.put(ArmCommand(arm=bool(arm)))

    def enqueue_takeoff(self, altitude_m: float) -> None:
        self._commands.put(TakeoffCommand(altitude_m=float(altitude_m)))

    def enqueue_velocity(self, command: VelocityCommand) -> None:
        self._commands.put(command)

    def snapshot(self) -> MavlinkSnapshot:
        with self._state_lock:
            return MavlinkSnapshot(
                heartbeat=dict(self._heartbeat) if self._heartbeat else None,
                local_position=self._local_position,
                attitude_quaternion_wxyz=self._attitude_quaternion_wxyz,
                error=self._error,
            )

    def _run(self) -> None:
        try:
            from pymavlink import mavutil

            self._mavutil = mavutil
            self._connection = mavutil.mavlink_connection(
                self._endpoint,
                dialect="ardupilotmega",
                autoreconnect=True,
            )
            self._logger.info(f"listening for MAVLink on {self._endpoint}")

            next_gcs_heartbeat_s = 0.0
            next_interval_request_s = 0.0
            while not self._stop_event.is_set():
                now_s = time.monotonic()
                if now_s >= next_gcs_heartbeat_s:
                    self._send_gcs_heartbeat()
                    next_gcs_heartbeat_s = now_s + 1.0

                if self._target_system and now_s >= next_interval_request_s:
                    self._request_message_intervals()
                    next_interval_request_s = now_s + 10.0

                self._drain_commands()

                message = self._connection.recv_match(blocking=True, timeout=0.1)
                if message is not None:
                    self._handle_message(message)
        except Exception as exc:  # pragma: no cover - exercised in live Docker runs.
            self._set_error(f"{type(exc).__name__}: {exc}")
            self._logger.error(f"MAVLink worker stopped: {type(exc).__name__}: {exc}")

    def _handle_message(self, message: Any) -> None:
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return
        if message_type not in self._seen_message_types:
            self._seen_message_types.add(message_type)
            self._logger.info(f"received MAVLink {message_type}")

        if message_type == "HEARTBEAT":
            self._handle_heartbeat(message)
            return

        if message_type == "COMMAND_ACK":
            self._logger.info(
                "received MAVLink COMMAND_ACK "
                f"command={message.command} result={message.result}"
            )
            return

        if message_type == "LOCAL_POSITION_NED":
            with self._state_lock:
                self._local_position = LocalPositionNed(
                    x=float(message.x),
                    y=float(message.y),
                    z=float(message.z),
                    vx=float(message.vx),
                    vy=float(message.vy),
                    vz=float(message.vz),
                )
            return

        if message_type == "GLOBAL_POSITION_INT":
            lat_e7 = int(message.lat)
            lon_e7 = int(message.lon)
            if lat_e7 == 0 and lon_e7 == 0:
                return
            local_position, origin = global_position_int_to_local_ned(
                GlobalPositionInt(
                    lat_e7=lat_e7,
                    lon_e7=lon_e7,
                    relative_alt_mm=int(message.relative_alt),
                    vx_cm_s=int(message.vx),
                    vy_cm_s=int(message.vy),
                    vz_cm_s=int(message.vz),
                ),
                self._global_origin,
            )
            with self._state_lock:
                self._global_origin = origin
                self._local_position = local_position
            return

        if message_type == "ATTITUDE":
            with self._state_lock:
                self._attitude_quaternion_wxyz = attitude_rpy_to_quaternion_wxyz(
                    message.roll,
                    message.pitch,
                    message.yaw,
                )
            return

        if message_type == "ATTITUDE_QUATERNION":
            with self._state_lock:
                self._attitude_quaternion_wxyz = (
                    float(message.q1),
                    float(message.q2),
                    float(message.q3),
                    float(message.q4),
                )

    def _handle_heartbeat(self, message: Any) -> None:
        mavutil = self._mavutil
        if mavutil is None or self._connection is None:
            return

        self._target_system = int(message.get_srcSystem())
        self._target_component = int(message.get_srcComponent())
        armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        heartbeat = {
            "sysid": self._target_system,
            "compid": self._target_component,
            "type": self._enum_label("MAV_TYPE", message.type),
            "autopilot": self._enum_label("MAV_AUTOPILOT", message.autopilot),
            "base_mode": int(message.base_mode),
            "custom_mode": int(message.custom_mode),
            "mode": getattr(self._connection, "flightmode", "UNKNOWN"),
            "system_status": self._enum_label("MAV_STATE", message.system_status),
            "armed": armed,
            "last_seen_monotonic_s": time.monotonic(),
        }
        with self._state_lock:
            self._heartbeat = heartbeat

    def _enum_label(self, enum_name: str, value: int) -> str:
        mavutil = self._mavutil
        if mavutil is None:
            return str(value)
        entry = mavutil.mavlink.enums.get(enum_name, {}).get(value)
        if entry is None:
            return str(value)
        return entry.name

    def _drain_commands(self) -> None:
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return

            if isinstance(command, ArmCommand):
                self._send_arm(command.arm)
            elif isinstance(command, TakeoffCommand):
                self._send_takeoff(command.altitude_m)
            elif isinstance(command, VelocityCommand):
                self._send_velocity(command)

    def _target_ready(self) -> bool:
        if self._connection is None or self._mavutil is None:
            return False
        if self._target_system:
            return True
        self._logger.warning("dropping MAVLink command until heartbeat sets target")
        return False

    def _send_gcs_heartbeat(self) -> None:
        if self._connection is None or self._mavutil is None:
            return
        mavlink = self._mavutil.mavlink
        self._connection.mav.heartbeat_send(
            mavlink.MAV_TYPE_GCS,
            mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
        )

    def _request_message_intervals(self) -> None:
        if not self._target_ready():
            return
        mavlink = self._mavutil.mavlink
        message_rates_hz = {
            mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10.0,
            mavlink.MAVLINK_MSG_ID_ATTITUDE: 10.0,
            mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION: 10.0,
            mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 2.0,
        }
        for message_id, rate_hz in message_rates_hz.items():
            self._connection.mav.command_long_send(
                self._target_system,
                self._target_component,
                mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                float(int(1_000_000 / rate_hz)),
                0,
                0,
                0,
                0,
                0,
            )
        for stream_id, rate_hz in (
            (mavlink.MAV_DATA_STREAM_POSITION, 10),
            (mavlink.MAV_DATA_STREAM_EXTRA1, 10),
        ):
            self._connection.mav.request_data_stream_send(
                self._target_system,
                self._target_component,
                stream_id,
                rate_hz,
                1,
            )

    def _send_arm(self, arm: bool) -> None:
        if not self._target_ready():
            return
        mavlink = self._mavutil.mavlink
        self._connection.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self._logger.info(f"sent MAVLink {'arm' if arm else 'disarm'} command")

    def _send_takeoff(self, altitude_m: float) -> None:
        if altitude_m <= 0.0:
            self._logger.warning("ignoring non-positive takeoff altitude")
            return
        if not self._target_ready():
            return
        self._ensure_guided_mode()
        mavlink = self._mavutil.mavlink
        self._connection.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            float(altitude_m),
        )
        self._logger.info(f"sent MAVLink takeoff command altitude_m={altitude_m:g}")

    def _send_velocity(self, command: VelocityCommand) -> None:
        if not self._target_ready():
            return
        self._ensure_guided_mode()
        mavlink = self._mavutil.mavlink
        time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        self._connection.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self._target_system,
            self._target_component,
            mavlink.MAV_FRAME_LOCAL_NED,
            LOCAL_NED_VELOCITY_TYPE_MASK,
            0,
            0,
            0,
            float(command.north_m_s),
            float(command.east_m_s),
            float(command.down_m_s),
            0,
            0,
            0,
            0,
            float(command.yaw_rate_rad_s),
        )

    def _ensure_guided_mode(self) -> None:
        if self._connection is None or self._mavutil is None:
            return
        now_s = time.monotonic()
        if now_s - self._last_guided_request_s < 2.0:
            return
        mode_mapping = self._connection.mode_mapping() or {}
        guided_mode = mode_mapping.get("GUIDED")
        if guided_mode is None:
            self._logger.warning("GUIDED mode is not available in MAVLink mode map")
            return
        self._connection.mav.set_mode_send(
            self._target_system,
            self._mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode,
        )
        self._last_guided_request_s = now_s

    def _set_error(self, error: str) -> None:
        with self._state_lock:
            self._error = error


class MavlinkBridgeNode(Node):
    """ROS 2 node for MAVLink telemetry topics and basic command topics."""

    def __init__(self) -> None:
        super().__init__("drone_mavlink_bridge")
        self.declare_parameter("connect", "udpin:0.0.0.0:14550")
        endpoint = (
            self.get_parameter("connect").get_parameter_value().string_value
            or "udpin:0.0.0.0:14550"
        )

        self._heartbeat_pub = self.create_publisher(String, "/drone/heartbeat", 10)
        self._odom_pub = self.create_publisher(Odometry, "/drone/odom", 10)
        self.create_subscription(Bool, "/drone/arm", self._on_arm, 10)
        self.create_subscription(
            Float32,
            "/drone/takeoff_alt_m",
            self._on_takeoff_altitude,
            10,
        )
        self.create_subscription(Twist, "/drone/cmd_vel", self._on_cmd_vel, 10)

        self._worker = MavlinkWorker(endpoint=endpoint, logger=self.get_logger())
        self._worker.start()
        self._last_error: str | None = None
        self.create_timer(0.1, self._publish_snapshot)

    def destroy_node(self) -> bool:
        self._worker.stop()
        return super().destroy_node()

    def _on_arm(self, message: Bool) -> None:
        self._worker.enqueue_arm(bool(message.data))

    def _on_takeoff_altitude(self, message: Float32) -> None:
        self._worker.enqueue_takeoff(float(message.data))

    def _on_cmd_vel(self, message: Twist) -> None:
        north_m_s, east_m_s, down_m_s = ros_enu_twist_to_mavlink_ned_velocity(
            (
                message.linear.x,
                message.linear.y,
                message.linear.z,
            )
        )
        self._worker.enqueue_velocity(
            VelocityCommand(
                north_m_s=north_m_s,
                east_m_s=east_m_s,
                down_m_s=down_m_s,
                yaw_rate_rad_s=ros_yaw_rate_to_mavlink_ned(message.angular.z),
            )
        )

    def _publish_snapshot(self) -> None:
        snapshot = self._worker.snapshot()
        if snapshot.error and snapshot.error != self._last_error:
            self._last_error = snapshot.error
            self.get_logger().error(f"MAVLink bridge error: {snapshot.error}")

        if snapshot.heartbeat is not None:
            self._heartbeat_pub.publish(
                String(data=json.dumps(snapshot.heartbeat, sort_keys=True))
            )

        if snapshot.local_position is not None:
            self._publish_odom(snapshot)

    def _publish_odom(self, snapshot: MavlinkSnapshot) -> None:
        if snapshot.local_position is None:
            return
        fields = local_position_ned_to_ros_odom_fields(
            snapshot.local_position,
            snapshot.attitude_quaternion_wxyz,
        )

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = fields.position_enu[0]
        odom.pose.pose.position.y = fields.position_enu[1]
        odom.pose.pose.position.z = fields.position_enu[2]
        odom.pose.pose.orientation.x = fields.orientation_xyzw[0]
        odom.pose.pose.orientation.y = fields.orientation_xyzw[1]
        odom.pose.pose.orientation.z = fields.orientation_xyzw[2]
        odom.pose.pose.orientation.w = fields.orientation_xyzw[3]
        odom.twist.twist.linear.x = fields.linear_velocity_enu[0]
        odom.twist.twist.linear.y = fields.linear_velocity_enu[1]
        odom.twist.twist.linear.z = fields.linear_velocity_enu[2]
        self._odom_pub.publish(odom)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
