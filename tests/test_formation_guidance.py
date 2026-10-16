import math

import pytest

from src.ros2.drone_bridge.drone_bridge.formation_guidance import (
    formation_command, target_body_point,
)
from src.ros2.drone_bridge.drone_bridge.scenario_controller import (
    ControllerConfig, MissionController, MissionState, Observation,
)


def config():
    return ControllerConfig(12.0, (-8.0, -5.0, 0.0), 120.0, 5.0, 10.0, 5.0, 3.0, 12.0, 145.0)


def observation(now, chaser, target=(20.0, 20.0, 20.0), yaw=0.0):
    return Observation(now, True, True, chaser, target, 0.0, yaw, (1.8, 0.0, 0.0))


def active_controller():
    controller = MissionController(config())
    controller.state = MissionState.INTERCEPT
    controller.state_entered_s = 0.0
    controller.mission_started_s = 0.0
    controller.spawn_z_m = 0.3
    controller.spawn_clearance_m = 0.3
    controller.clearance_enforced = True
    return controller


def test_target_body_offset_rotates_with_target_heading():
    point = target_body_point((10.0, 20.0, 30.0), math.pi / 2.0, (-8.0, -5.0, 2.0))
    assert point == pytest.approx((15.0, 12.0, 32.0))


def test_command_includes_target_feed_forward_and_is_bounded():
    at_formation = formation_command((12.0, 15.0, 20.0), (20.0, 20.0, 20.0), 0.0, (-8.0, -5.0, 0.0), 12.0, target_velocity_xyz=(1.8, 0.0, 0.0))
    assert at_formation.error_m == 0.0
    assert at_formation.velocity_xyz == pytest.approx((1.8, 0.0, 0.0))
    far = formation_command((-100.0, -100.0, 0.0), (20.0, 20.0, 20.0), 0.0, (-8.0, -5.0, 0.0), 12.0)
    assert math.dist((0.0, 0.0, 0.0), far.velocity_xyz) == pytest.approx(12.0)


def test_threshold_loss_reacquisition_requires_fresh_continuous_hold():
    controller = active_controller()
    desired = (12.0, 15.0, 20.0)
    assert controller.tick(observation(1.0, desired)).state == MissionState.FORMATION_HOLD
    assert controller.tick(observation(8.0, desired)).state == MissionState.FORMATION_HOLD

    lost = controller.tick(observation(9.0, (0.0, 0.0, 20.0)))
    assert lost.state == MissionState.INTERCEPT
    assert controller.hold_started_s is None
    assert math.dist((0.0, 0.0, 0.0), lost.velocity_xyz) <= 12.0

    assert controller.tick(observation(10.0, desired)).state == MissionState.FORMATION_HOLD
    assert controller.tick(observation(19.9, desired)).state == MissionState.FORMATION_HOLD
    complete = controller.tick(observation(20.0, desired))
    assert complete.state == MissionState.COMPLETE
    assert complete.velocity_xyz == pytest.approx((1.8, 0.0, 0.0))
