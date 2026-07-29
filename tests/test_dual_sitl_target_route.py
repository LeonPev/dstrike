from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.dual_sitl_target_probe import (
    EndpointReversalTracker,
    ProbeFailure,
    odom_sample,
)
from src.physics.scenario_config import TargetDroneConfig
from src.physics.target_motion import TargetMotion
from src.physics.terrain_map import TerrainMap
from src.ros2.drone_bridge.drone_bridge.target_route_controller import (
    ARM_TOPIC,
    CONTROL_PERIOD_S,
    FLIGHT_RELEASE_TOPIC,
    HEARTBEAT_TOPIC,
    ODOMETRY_TOPIC,
    ROUTE_STATE_TOPIC,
    TAKEOFF_TOPIC,
    VELOCITY_TOPIC,
    TargetRouteAction,
    TargetRouteControlConfig,
    TargetRouteController,
    TargetRouteObservation,
    TargetRouteState,
    load_canonical_controller,
)


ROOT = Path(__file__).resolve().parents[1]


def test_probe_rejects_non_finite_yaw_odometry() -> None:
    message = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=10.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=math.nan),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )
        ),
    )

    with pytest.raises(ProbeFailure, match="non-finite.*yaw"):
        odom_sample(message)


def flat_terrain() -> TerrainMap:
    return TerrainMap(np.zeros((3, 3), dtype=np.float32), 100.0)


def rising_terrain() -> TerrainMap:
    # Height rises with ENU +X and is constant along Y.
    return TerrainMap(
        np.array(
            [
                [0.0, 5.0, 10.0],
                [0.0, 5.0, 10.0],
                [0.0, 5.0, 10.0],
            ],
            dtype=np.float32,
        ),
        100.0,
    )


def controller(
    *,
    terrain: TerrainMap | None = None,
    **config_overrides: float,
) -> tuple[TargetRouteController, TargetMotion, TerrainMap]:
    terrain = terrain or flat_terrain()
    target = TargetDroneConfig(
        name="target",
        start_xy_m=(-10.0, 0.0),
        heading_deg=0.0,
        speed_mps=2.0,
        altitude_agl_m=10.0,
        route_length_m=20.0,
        route_behavior="ping_pong",
    )
    motion = TargetMotion(target, terrain)
    values = {
        "target_agl_m": 10.0,
        "geofence_radius_m": 45.0,
        "min_clearance_m": 2.0,
        "activation_takeoff_alt_m": 0.5,
        "post_release_takeoff_s": 0.75,
        "health_timeout_s": 2.0,
        "arm_timeout_s": 2.0,
        "stabilize_timeout_s": 4.0,
        "stabilize_duration_s": 1.0,
        "telemetry_stale_s": 0.5,
        "stabilize_xy_error_m": 1.0,
        "stabilize_speed_mps": 0.5,
        "stabilize_agl_error_m": 0.5,
        "stabilize_yaw_error_rad": 0.2,
        "position_gain_s": 0.5,
        "agl_gain_s": 0.6,
        "yaw_gain_s": 2.0,
        "max_horizontal_speed_mps": 3.0,
        "max_vertical_speed_mps": 1.0,
        "max_yaw_rate_rad_s": 0.5,
        "terrain_feedforward_step_s": 0.1,
    }
    values.update(config_overrides)
    core = TargetRouteController(TargetRouteControlConfig(**values), motion, terrain)
    return core, motion, terrain


def observation(
    now_s: float,
    *,
    healthy: bool = True,
    armed: bool = False,
    position_xyz: tuple[float, float, float] | None = (-10.0, 0.0, 10.0),
    velocity_xyz: tuple[float, float, float] | None = (0.0, 0.0, 0.0),
    yaw_rad: float | None = 0.0,
    heartbeat_age_s: float = 0.1,
    odometry_age_s: float = 0.1,
) -> TargetRouteObservation:
    return TargetRouteObservation(
        now_s=now_s,
        healthy=healthy,
        armed=armed,
        position_xyz=position_xyz,
        velocity_xyz=velocity_xyz,
        yaw_rad=yaw_rad,
        heartbeat_age_s=heartbeat_age_s,
        odometry_age_s=odometry_age_s,
    )


def test_dynamic_endpoint_reversal_settles_after_leaving_endpoint_band() -> None:
    tracker = EndpointReversalTracker(route_length_m=240.0, target_speed_mps=1.8)

    assert not tracker.observe(
        sample_s=100.0,
        along_m=238.0,
        cross_track_m=0.05,
        reverse_velocity_error_mps=3.5,
        agl_error_m=0.1,
        reverse_heading_error_deg=179.0,
    )
    assert tracker.observe(
        sample_s=104.0,
        along_m=231.0,
        cross_track_m=0.1,
        reverse_velocity_error_mps=0.05,
        agl_error_m=0.2,
        reverse_heading_error_deg=2.0,
    )
    assert tracker.max_along_m == 238.0


def test_dynamic_endpoint_reversal_enforces_overshoot_and_timeout() -> None:
    overshoot = EndpointReversalTracker(route_length_m=20.0, target_speed_mps=2.0)
    with pytest.raises(ProbeFailure, match="overshot"):
        overshoot.observe(
            sample_s=1.0,
            along_m=23.01,
            cross_track_m=0.0,
            reverse_velocity_error_mps=4.0,
            agl_error_m=0.0,
            reverse_heading_error_deg=180.0,
        )

    timeout = EndpointReversalTracker(route_length_m=20.0, target_speed_mps=2.0)
    assert not timeout.observe(
        sample_s=10.0,
        along_m=18.0,
        cross_track_m=0.0,
        reverse_velocity_error_mps=4.0,
        agl_error_m=0.0,
        reverse_heading_error_deg=180.0,
    )
    with pytest.raises(ProbeFailure, match="within 10.0 s"):
        timeout.observe(
            sample_s=20.01,
            along_m=12.0,
            cross_track_m=0.0,
            reverse_velocity_error_mps=0.0,
            agl_error_m=0.0,
            reverse_heading_error_deg=0.0,
        )


def enter_route(
    core: TargetRouteController,
    *,
    start_s: float = 0.0,
    position_xyz: tuple[float, float, float] = (-10.0, 0.0, 10.0),
    yaw_rad: float = 0.0,
) -> TargetRouteAction:
    assert core.tick(observation(start_s, position_xyz=position_xyz)).state == TargetRouteState.ARM
    assert core.tick(
        observation(
            start_s + 0.1,
            armed=True,
            position_xyz=position_xyz,
            yaw_rad=yaw_rad,
        )
    ).state == TargetRouteState.STABILIZE
    core.tick(
        observation(
            start_s + 0.2,
            armed=True,
            position_xyz=position_xyz,
            yaw_rad=yaw_rad,
        )
    )
    return core.tick(
        observation(
            start_s + 1.21,
            armed=True,
            position_xyz=position_xyz,
            yaw_rad=yaw_rad,
        )
    )


def test_lifecycle_releases_only_after_health_arm_and_continuous_stabilization() -> None:
    core, _, _ = controller()

    waiting = core.tick(
        observation(
            0.0,
            healthy=False,
            position_xyz=None,
            velocity_xyz=None,
            yaw_rad=None,
            heartbeat_age_s=math.inf,
            odometry_age_s=math.inf,
        )
    )
    assert waiting.state == TargetRouteState.WAIT_HEALTH
    assert not waiting.request_arm
    assert waiting.velocity_enu is None
    assert not waiting.release_flight

    arming = core.tick(observation(0.1))
    assert arming.state == TargetRouteState.ARM
    assert arming.request_arm
    assert not arming.release_flight

    stabilizing = core.tick(observation(0.2, armed=True))
    assert stabilizing.state == TargetRouteState.STABILIZE
    assert stabilizing.velocity_enu is None
    assert stabilizing.request_takeoff_alt_m == 0.5
    assert not stabilizing.release_flight

    assert not core.tick(observation(0.3, armed=True)).release_flight
    released = core.tick(observation(1.31, armed=True))
    assert released.state == TargetRouteState.ROUTE
    assert released.release_flight
    assert core.flight_released
    assert released.velocity_enu is None

    continuing = core.tick(observation(1.41, armed=True))
    assert continuing.state == TargetRouteState.ROUTE
    assert not continuing.release_flight
    assert continuing.velocity_enu is None

    active = core.tick(observation(2.1, armed=True))
    assert active.velocity_enu is not None


def test_stabilization_must_remain_bounded_for_the_whole_window() -> None:
    core, _, _ = controller()
    core.tick(observation(0.0))
    core.tick(observation(0.1, armed=True))
    core.tick(observation(0.2, armed=True))

    disturbed = core.tick(
        observation(0.8, armed=True, position_xyz=(-8.0, 0.0, 10.0))
    )
    assert disturbed.state == TargetRouteState.STABILIZE
    assert not disturbed.release_flight

    assert not core.tick(observation(1.0, armed=True)).release_flight
    released = core.tick(observation(2.01, armed=True))
    assert released.state == TargetRouteState.ROUTE
    assert released.release_flight


@pytest.mark.parametrize(
    ("setup", "unsafe", "reason"),
    [
        (
            lambda core: core.tick(observation(0.0)),
            observation(0.6, heartbeat_age_s=0.6),
            "heartbeat stale",
        ),
        (
            lambda core: None,
            observation(0.0, position_xyz=(46.0, 0.0, 10.0)),
            "geofence",
        ),
        (
            lambda core: None,
            observation(0.0, position_xyz=(-10.0, 0.0, 1.0)),
            "clearance",
        ),
    ],
)
def test_stale_geofence_and_clearance_abort_without_release(
    setup, unsafe: TargetRouteObservation, reason: str
) -> None:
    core, _, _ = controller()
    setup(core)
    action = core.tick(unsafe)

    assert action.state == TargetRouteState.ABORT
    assert action.velocity_enu == (0.0, 0.0, 0.0)
    assert not action.release_flight
    assert reason in (action.abort_reason or "")
    assert not core.flight_released


def test_each_pre_route_wait_has_a_finite_timeout_and_zero_abort_command() -> None:
    core, _, _ = controller()
    core.tick(
        observation(
            0.0,
            healthy=False,
            position_xyz=None,
            velocity_xyz=None,
            yaw_rad=None,
            heartbeat_age_s=math.inf,
            odometry_age_s=math.inf,
        )
    )
    health_timeout = core.tick(
        observation(
            2.1,
            healthy=False,
            position_xyz=None,
            velocity_xyz=None,
            yaw_rad=None,
            heartbeat_age_s=math.inf,
            odometry_age_s=math.inf,
        )
    )
    assert health_timeout.state == TargetRouteState.ABORT
    assert health_timeout.abort_reason == "health timeout"
    assert health_timeout.velocity_enu == (0.0, 0.0, 0.0)

    core, _, _ = controller()
    core.tick(observation(0.0))
    arm_timeout = core.tick(observation(2.1))
    assert arm_timeout.state == TargetRouteState.ABORT
    assert arm_timeout.abort_reason == "arm timeout"

    core, _, _ = controller()
    core.tick(observation(0.0))
    core.tick(observation(0.1, armed=True))
    stabilize_timeout = core.tick(
        observation(
            4.2,
            armed=True,
            position_xyz=(-8.0, 0.0, 10.0),
        )
    )
    assert stabilize_timeout.state == TargetRouteState.ABORT
    assert stabilize_timeout.abort_reason == "stabilization timeout"


def test_route_command_is_bounded_in_enu_and_uses_bounded_yaw_rate() -> None:
    core, _, _ = controller()
    released = enter_route(core)
    assert released.state == TargetRouteState.ROUTE

    action = core.tick(
        observation(
            2.1,
            armed=True,
            position_xyz=(-40.0, 20.0, 14.0),
            yaw_rad=math.pi,
        )
    )
    assert action.velocity_enu is not None
    assert math.hypot(*action.velocity_enu[:2]) <= 3.0
    assert abs(action.velocity_enu[2]) <= 1.0
    assert abs(action.yaw_rate_rad_s) <= 0.5
    assert all(math.isfinite(value) for value in action.velocity_enu)


def test_route_yaw_deadband_does_not_chase_sub_degree_noise() -> None:
    core, motion, _ = controller(yaw_deadband_rad=math.radians(1.0))
    enter_route(core)
    route_start_s = core.route_started_s
    assert route_start_s is not None
    desired = motion.state_at(1.0)

    action = core.tick(
        observation(
            route_start_s + 1.0,
            armed=True,
            position_xyz=desired.position_xyz,
            velocity_xyz=desired.velocity_xyz,
            yaw_rad=math.radians(desired.heading_deg + 0.5),
        )
    )
    assert action.yaw_rate_rad_s == 0.0


def test_route_uses_ping_pong_reference_and_reverses_feedforward() -> None:
    core, motion, _ = controller()
    released = enter_route(core)
    route_start_s = core.route_started_s
    assert route_start_s is not None
    assert released.reference is not None and released.reference.direction == 1

    elapsed_s = 10.1
    desired = motion.state_at(elapsed_s)
    action = core.tick(
        observation(
            route_start_s + elapsed_s,
            armed=True,
            position_xyz=desired.position_xyz,
            velocity_xyz=desired.velocity_xyz,
            yaw_rad=math.radians(desired.heading_deg),
        )
    )
    assert action.reference is not None
    assert action.reference.direction == -1
    assert action.reference.heading_deg == 180.0
    assert action.velocity_enu is not None and action.velocity_enu[0] < 0.0
    assert abs(action.velocity_enu[1]) < 1e-9


def test_agl_error_and_terrain_slope_drive_bounded_vertical_correction() -> None:
    core, motion, terrain = controller(terrain=rising_terrain())
    start = motion.state_at(0.0)
    released = enter_route(
        core,
        position_xyz=start.position_xyz,
        yaw_rad=math.radians(start.heading_deg),
    )
    assert released.state == TargetRouteState.ROUTE
    route_start_s = core.route_started_s
    assert route_start_s is not None

    elapsed_s = 1.0
    desired = motion.state_at(elapsed_s)
    terrain_z = terrain.height_at(*desired.position_xyz[:2])
    below = core.tick(
        observation(
            route_start_s + elapsed_s,
            armed=True,
            position_xyz=(desired.position_xyz[0], desired.position_xyz[1], terrain_z + 8.0),
            yaw_rad=math.radians(desired.heading_deg),
        )
    )
    assert below.velocity_enu is not None
    assert 0.0 < below.velocity_enu[2] <= 1.0

    above = core.tick(
        observation(
            route_start_s + elapsed_s + 0.1,
            armed=True,
            position_xyz=(desired.position_xyz[0], desired.position_xyz[1], terrain_z + 14.0),
            yaw_rad=math.radians(desired.heading_deg),
        )
    )
    assert above.velocity_enu is not None
    assert -1.0 <= above.velocity_enu[2] < 0.0


def test_route_safety_failure_aborts_with_zero_velocity_and_never_releases_again() -> None:
    core, _, _ = controller()
    assert enter_route(core).release_flight

    aborted = core.tick(
        observation(1.4, armed=False)
    )
    assert aborted.state == TargetRouteState.ABORT
    assert aborted.velocity_enu == (0.0, 0.0, 0.0)
    assert not aborted.release_flight
    assert "disarmed" in (aborted.abort_reason or "")

    repeated = core.tick(observation(1.5, armed=True))
    assert repeated.state == TargetRouteState.ABORT
    assert repeated.velocity_enu == (0.0, 0.0, 0.0)
    assert not repeated.release_flight


def test_canonical_loader_uses_committed_scenario_terrain_and_route() -> None:
    core = load_canonical_controller(ROOT)
    initial = core.reference.state_at(0.0)
    later = core.reference.state_at(10.0)

    assert core.config.target_agl_m == 35.0
    assert core.config.min_clearance_m == 5.0
    assert core.config.arm_timeout_s == 75.0
    assert core.config.stabilize_duration_s == 8.0
    assert core.config.activation_takeoff_alt_m == 0.5
    assert core.config.post_release_takeoff_s == 0.75
    assert core.config.stabilize_xy_error_m == 0.25
    assert core.config.stabilize_agl_error_m == 0.75
    assert core.config.stabilize_speed_mps == 0.25
    assert core.config.yaw_gain_s == 0.6
    assert core.config.yaw_deadband_rad == math.radians(1.0)
    assert core.config.max_yaw_rate_rad_s == math.radians(45.0)
    assert initial.position_xyz[:2] == (-110.0, 0.0)
    assert later.position_xyz[0] > initial.position_xyz[0]
    assert math.isclose(
        initial.position_xyz[2]
        - core.terrain.height_at(*initial.position_xyz[:2]),
        35.0,
    )


def test_dual_launch_and_relative_topic_contract_are_wired() -> None:
    launch = (
        ROOT / "src/ros2/drone_bridge/launch/dual_sitl.launch.py"
    ).read_text(encoding="utf-8")
    setup = (ROOT / "src/ros2/drone_bridge/setup.py").read_text(encoding="utf-8")

    assert 'executable="target_route_controller"' in launch
    assert "namespace=target_route.namespace" in launch
    assert "target_route_controller = drone_bridge.target_route_controller:main" in setup
    assert HEARTBEAT_TOPIC == "drone/heartbeat"
    assert ODOMETRY_TOPIC == "drone/odom"
    assert ARM_TOPIC == "drone/arm"
    assert TAKEOFF_TOPIC == "drone/takeoff_alt_m"
    assert VELOCITY_TOPIC == "drone/cmd_vel"
    assert ROUTE_STATE_TOPIC == "route/state"
    assert FLIGHT_RELEASE_TOPIC == "/sim/target/flight_release"
    assert CONTROL_PERIOD_S < 0.75
