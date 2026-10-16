from __future__ import annotations

import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src" / "ros2" / "drone_bridge"),
)

from drone_bridge.mavlink_bridge import (  # noqa: E402
    ARDUPILOT_FORCE_ARM_MAGIC,
    ARDUPILOT_FORCE_DISARM_MAGIC,
    MavlinkWorker,
    SimStateOrigin,
    VELOCITY_COMMAND_TIMEOUT_S,
    VELOCITY_ZERO_HOLD_AFTER_TIMEOUT_S,
    VelocityCommand,
    apply_odom_position_offset,
    odom_alignment_offset,
    sim_state_to_local_position_ned,
)
from drone_bridge.frame_conventions import (  # noqa: E402
    EARTH_RADIUS_M,
    GlobalPositionOrigin,
    RosOdomFields,
)


def test_sim_state_converts_sitl_truth_to_origin_relative_ned() -> None:
    origin = SimStateOrigin(
        origin=GlobalPositionOrigin(lat_deg=-35.0, lon_deg=149.0),
        altitude_m=584.0,
    )
    latitude = origin.origin.lat_deg + math.degrees(10.0 / EARTH_RADIUS_M)
    longitude = origin.origin.lon_deg + math.degrees(
        20.0 / (EARTH_RADIUS_M * math.cos(math.radians(origin.origin.lat_deg)))
    )
    message = SimpleNamespace(
        lat_int=int(round(latitude * 10_000_000)),
        lon_int=int(round(longitude * 10_000_000)),
        alt=587.0,
        vn=1.25,
        ve=-2.5,
        vd=0.75,
    )

    position = sim_state_to_local_position_ned(message, origin)

    assert math.isclose(position.x, 10.0, abs_tol=0.02)
    assert math.isclose(position.y, 20.0, abs_tol=0.02)
    assert position.z == -3.0
    assert (position.vx, position.vy, position.vz) == (1.25, -2.5, 0.75)


def test_sim_state_worker_uses_first_truth_sample_as_fixed_origin() -> None:
    logger = FakeLogger()
    worker = MavlinkWorker(
        endpoint="udpin:0.0.0.0:14550",
        logger=logger,
        use_sim_state=True,
    )
    first_latitude = -35.0
    second_latitude = first_latitude + math.degrees(1.0 / EARTH_RADIUS_M)

    def message(latitude: float) -> SimpleNamespace:
        return SimpleNamespace(
            get_type=lambda: "SIM_STATE",
            lat_int=int(round(latitude * 10_000_000)),
            lon_int=1_490_000_000,
            alt=584.0,
            vn=0.0,
            ve=0.0,
            vd=0.0,
            q1=1.0,
            q2=0.0,
            q3=0.0,
            q4=0.0,
        )

    worker._handle_message(message(first_latitude))
    first = worker.snapshot()
    worker._handle_message(message(second_latitude))
    second = worker.snapshot()

    assert first.local_position is not None
    assert math.isclose(first.local_position.x, 0.0, abs_tol=0.02)
    assert second.local_position is not None
    assert math.isclose(second.local_position.x, 1.0, abs_tol=0.02)


def test_configured_odom_offset_aligns_local_pose_to_world() -> None:
    fields = RosOdomFields(
        position_enu=(0.25, -0.5, -0.3),
        linear_velocity_enu=(1.0, 2.0, 3.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    aligned = apply_odom_position_offset(fields, (-120.0, -25.0, 16.2))

    assert all(
        math.isclose(actual, expected, abs_tol=1e-12)
        for actual, expected in zip(
            aligned.position_enu,
            (-119.75, -25.5, 15.9),
        )
    )
    assert aligned.linear_velocity_enu == fields.linear_velocity_enu
    assert aligned.orientation_xyzw == fields.orientation_xyzw


def test_alignment_offset_registers_current_local_pose_to_world_reference() -> None:
    fields = RosOdomFields(
        position_enu=(0.25, -0.5, -0.3),
        linear_velocity_enu=(1.0, 2.0, 3.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    reference = (-119.75, -25.5, 15.9)

    offset = odom_alignment_offset(fields, reference)
    aligned = apply_odom_position_offset(fields, offset)

    assert all(
        math.isclose(actual, expected, abs_tol=1e-12)
        for actual, expected in zip(aligned.position_enu, reference)
    )


class FakeLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def info(self, message: str) -> None:
        pass

    def warning(self, message: str) -> None:
        self.warnings.append(message)

    def error(self, message: str) -> None:
        pass


class FakeMavlink:
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_CMD_NAV_LAND = 21
    MAV_FRAME_LOCAL_NED = 1
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1


class FakeMavutil:
    mavlink = FakeMavlink()


class FakeMav:
    def __init__(self) -> None:
        self.command_long_calls: list[tuple[object, ...]] = []
        self.set_position_target_local_ned_calls: list[tuple[object, ...]] = []
        self.set_mode_calls: list[tuple[object, ...]] = []

    def command_long_send(self, *args: object) -> None:
        self.command_long_calls.append(args)

    def set_position_target_local_ned_send(self, *args: object) -> None:
        self.set_position_target_local_ned_calls.append(args)

    def set_mode_send(self, *args: object) -> None:
        self.set_mode_calls.append(args)


class FakeConnection:
    def __init__(self) -> None:
        self.mav = FakeMav()

    def mode_mapping(self) -> dict[str, int]:
        return {"GUIDED": 4}


def ready_worker() -> tuple[MavlinkWorker, FakeConnection, FakeLogger]:
    logger = FakeLogger()
    worker = MavlinkWorker(endpoint="udpin:0.0.0.0:14550", logger=logger)
    connection = FakeConnection()
    worker._connection = connection
    worker._mavutil = FakeMavutil()
    worker._target_system = 1
    worker._target_component = 1
    worker._heartbeat = {"mode": "GUIDED"}
    return worker, connection, logger


def test_pending_arm_waits_for_local_position_ned() -> None:
    worker, connection, _ = ready_worker()
    worker._pending_arm = True
    worker._pending_arm_deadline_s = time.monotonic() + 60.0

    worker._try_send_pending_arm()

    assert worker._pending_arm is True
    assert connection.mav.command_long_calls == []

    worker._has_local_position_ned = True
    worker._try_send_pending_arm()

    assert worker._pending_arm is False
    assert len(connection.mav.command_long_calls) == 1
    arm_call = connection.mav.command_long_calls[0]
    assert arm_call[2] == FakeMavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert arm_call[4] == 1.0
    assert arm_call[5] == ARDUPILOT_FORCE_ARM_MAGIC


def test_normal_disarm_keeps_ardupilot_safety_checks() -> None:
    worker, connection, _ = ready_worker()

    worker._send_arm(False)

    assert len(connection.mav.command_long_calls) == 1
    disarm_call = connection.mav.command_long_calls[0]
    assert disarm_call[2] == FakeMavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert disarm_call[4] == 0.0
    assert disarm_call[5] == 0.0


def test_explicit_force_disarm_uses_ardupilot_magic() -> None:
    worker, connection, _ = ready_worker()

    worker.enqueue_force_disarm()
    worker._drain_commands()

    assert len(connection.mav.command_long_calls) == 1
    disarm_call = connection.mav.command_long_calls[0]
    assert disarm_call[2] == FakeMavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert disarm_call[4] == 0.0
    assert disarm_call[5] == ARDUPILOT_FORCE_DISARM_MAGIC


def test_repeated_land_commands_do_not_reenter_guided_mode() -> None:
    worker, connection, _ = ready_worker()
    worker._heartbeat = {"mode": "LAND"}

    worker._send_land()
    worker._send_land()

    assert connection.mav.set_mode_calls == []
    assert len(connection.mav.command_long_calls) == 2
    assert all(
        land_call[2] == FakeMavlink.MAV_CMD_NAV_LAND
        for land_call in connection.mav.command_long_calls
    )


def test_velocity_command_is_sent_as_guided_local_ned_setpoint() -> None:
    worker, connection, _ = ready_worker()

    worker._queue_velocity(
        VelocityCommand(
            north_m_s=1.0,
            east_m_s=2.0,
            down_m_s=-0.5,
            yaw_rate_rad_s=0.25,
        )
    )

    assert len(connection.mav.set_position_target_local_ned_calls) == 1
    velocity_call = connection.mav.set_position_target_local_ned_calls[0]
    assert velocity_call[8] == 1.0
    assert velocity_call[9] == 2.0
    assert velocity_call[10] == -0.5
    assert velocity_call[15] == 0.25


def test_stale_velocity_command_sends_zero_hold() -> None:
    worker, connection, logger = ready_worker()
    worker._queue_velocity(
        VelocityCommand(
            north_m_s=1.0,
            east_m_s=2.0,
            down_m_s=-0.5,
            yaw_rate_rad_s=0.25,
        )
    )

    now_s = time.monotonic()
    worker._active_velocity_received_s = now_s - VELOCITY_COMMAND_TIMEOUT_S - 0.1
    worker._active_velocity_sent_s = 0.0
    worker._refresh_velocity_command(now_s)

    zero_call = connection.mav.set_position_target_local_ned_calls[-1]
    assert zero_call[8] == 0.0
    assert zero_call[9] == 0.0
    assert zero_call[10] == 0.0
    assert zero_call[15] == 0.0
    assert logger.warnings == ["velocity command timed out; sending zero velocity hold"]


def test_zero_velocity_hold_clears_after_hold_window() -> None:
    worker, connection, _ = ready_worker()
    worker._queue_velocity(
        VelocityCommand(
            north_m_s=0.0,
            east_m_s=0.0,
            down_m_s=0.0,
            yaw_rate_rad_s=0.0,
        )
    )
    first_received_s = worker._active_velocity_received_s

    stale_s = first_received_s + VELOCITY_COMMAND_TIMEOUT_S + 0.1
    worker._active_velocity_sent_s = 0.0
    worker._refresh_velocity_command(stale_s)

    assert worker._active_velocity_command is not None
    assert len(connection.mav.set_position_target_local_ned_calls) == 2

    clear_s = first_received_s + VELOCITY_ZERO_HOLD_AFTER_TIMEOUT_S + 0.1
    worker._refresh_velocity_command(clear_s)

    assert worker._active_velocity_command is None
