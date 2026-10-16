import math

from src.ros2.drone_bridge.drone_bridge.scenario_controller import (
    ControllerConfig, MissionController, MissionState, Observation,
)


def config(**overrides):
    values = dict(
        takeoff_agl_m=12.0,
        formation_offset_m=(-8.0, -5.0, 0.0),
        mission_timeout_s=120.0,
        formation_threshold_m=5.0,
        formation_hold_s=10.0,
        min_clearance_m=5.0,
        min_separation_m=3.0,
        max_speed_mps=12.0,
        geofence_radius_m=145.0,
        state_timeout_s=60.0,
    )
    values.update(overrides)
    return ControllerConfig(**values)


def obs(now, chaser=(-120.0, -25.0, 10.3), target=(-100.0, 0.0, 35.0), terrain=0.0, healthy=True, armed=True):
    return Observation(now, healthy, armed, chaser, target, terrain)


def test_ground_clearance_exemption_and_takeoff_uses_spawn_relative_gain():
    core = MissionController(config())
    core.start(0.0)
    assert core.tick(obs(0.1, chaser=(-120.0, -25.0, 0.3))).state == MissionState.ARM
    action = core.tick(obs(0.2, chaser=(-120.0, -25.0, 0.3), armed=False))
    assert action.state == MissionState.ARM and action.arm
    action = core.tick(obs(0.3, chaser=(-120.0, -25.0, 0.3), armed=True))
    assert action.state == MissionState.TAKEOFF
    assert not action.arm and action.takeoff_alt_m == 12.0
    assert not core.clearance_enforced
    assert core.tick(obs(1.0, chaser=(-120.0, -25.0, 4.9))).state == MissionState.TAKEOFF
    assert core.tick(obs(2.0, chaser=(-120.0, -25.0, 11.4))).state == MissionState.INTERCEPT
    assert core.clearance_enforced


def test_intercept_requires_safe_clearance():
    core = MissionController(config(min_clearance_m=11.5))
    core.start(0.0)
    core.tick(obs(0.1, chaser=(-120.0, -25.0, 0.3)))
    core.tick(obs(0.2, chaser=(-120.0, -25.0, 0.3), armed=True))
    assert core.tick(obs(2.0, chaser=(-120.0, -25.0, 11.4))).state == MissionState.ABORT


def test_formation_hold_loss_resets_timer_and_keeps_command_bounded():
    core = MissionController(config())
    core.state = MissionState.INTERCEPT
    core.state_entered_s = 0.0
    core.mission_started_s = 0.0
    core.spawn_z_m = 0.3
    core.spawn_clearance_m = 0.3
    core.clearance_enforced = True
    target = (20.0, 20.0, 20.0)
    desired = (12.0, 15.0, 20.0)
    assert core.tick(obs(1.0, chaser=desired, target=target)).state == MissionState.FORMATION_HOLD
    assert core.hold_started_s == 1.0
    action = core.tick(obs(5.0, chaser=(0.0, 0.0, 20.0), target=target))
    assert action.state == MissionState.INTERCEPT
    assert core.hold_started_s is None
    assert math.dist((0.0, 0.0, 0.0), action.velocity_xyz) <= core.config.max_speed_mps


def test_active_flight_safety_failures_abort():
    for observation, reason in [
        (obs(1.0, chaser=(146.0, 0.0, 20.0)), "geofence"),
        (obs(1.0, chaser=(0.0, 0.0, 4.0)), "clearance"),
        (obs(1.0, chaser=(0.0, 0.0, 20.0), target=(1.0, 0.0, 20.0)), "separation"),
    ]:
        core = MissionController(config())
        core.state = MissionState.INTERCEPT
        core.mission_started_s = 0.0
        core.clearance_enforced = True
        action = core.tick(observation)
        assert action.state == MissionState.ABORT
        assert reason in action.abort_reason


def test_each_preflight_state_has_finite_timeout():
    core = MissionController(config(state_timeout_s=2.0))
    core.start(0.0)
    action = core.tick(obs(2.1, healthy=False, armed=False, chaser=None, target=None, terrain=None))
    assert action.state == MissionState.ABORT
    assert action.abort_reason == "HEALTH_CHECK timeout"
