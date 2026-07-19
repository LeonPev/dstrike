from __future__ import annotations

from pathlib import Path
import sys
import time


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src" / "ros2" / "drone_bridge"),
)

from drone_bridge.mavlink_bridge import (  # noqa: E402
    ARDUPILOT_FORCE_ARM_MAGIC,
    ARDUPILOT_FORCE_DISARM_MAGIC,
    MavlinkWorker,
    VELOCITY_COMMAND_TIMEOUT_S,
    VELOCITY_ZERO_HOLD_AFTER_TIMEOUT_S,
    VelocityCommand,
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
    MAV_FRAME_LOCAL_NED = 1


class FakeMavutil:
    mavlink = FakeMavlink()


class FakeMav:
    def __init__(self) -> None:
        self.command_long_calls: list[tuple[object, ...]] = []
        self.set_position_target_local_ned_calls: list[tuple[object, ...]] = []

    def command_long_send(self, *args: object) -> None:
        self.command_long_calls.append(args)

    def set_position_target_local_ned_send(self, *args: object) -> None:
        self.set_position_target_local_ned_calls.append(args)


class FakeConnection:
    def __init__(self) -> None:
        self.mav = FakeMav()


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
