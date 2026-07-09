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
    MavlinkWorker,
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


class FakeMavutil:
    mavlink = FakeMavlink()


class FakeMav:
    def __init__(self) -> None:
        self.command_long_calls: list[tuple[object, ...]] = []

    def command_long_send(self, *args: object) -> None:
        self.command_long_calls.append(args)


class FakeConnection:
    def __init__(self) -> None:
        self.mav = FakeMav()


def test_pending_arm_waits_for_local_position_ned() -> None:
    logger = FakeLogger()
    worker = MavlinkWorker(endpoint="udpin:0.0.0.0:14550", logger=logger)
    connection = FakeConnection()
    worker._connection = connection
    worker._mavutil = FakeMavutil()
    worker._target_system = 1
    worker._target_component = 1
    worker._heartbeat = {"mode": "GUIDED"}
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
