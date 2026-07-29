from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest

from src.physics.multi_vehicle_json import DUAL_SITL_CHANNELS, SharedMuJoCoWorld, validate_channels
from src.physics.ros_camera import camera_configs_for_scenario


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ros2" / "drone_bridge"))
from drone_bridge.multi_vehicle_bridge import DUAL_SITL_ROUTES, validate_routes


def test_json_channels_have_unique_ports_sysids_and_bodies():
    channels = validate_channels(DUAL_SITL_CHANNELS)
    assert {(item.role, item.port, item.sysid, item.body_name) for item in channels} == {
        ("target", 9003, 2, "target"),
        ("chaser", 9002, 1, "chaser"),
    }


def test_bridge_routes_have_separate_namespaces_and_topics():
    routes = validate_routes(DUAL_SITL_ROUTES)
    assert {route.heartbeat_topic for route in routes} == {"/target/drone/heartbeat", "/chaser/drone/heartbeat"}
    assert {route.odom_topic for route in routes} == {"/target/drone/odom", "/chaser/drone/odom"}
    assert {route.connect for route in routes} == {"udpin:0.0.0.0:14550", "udpin:0.0.0.0:14551"}


@pytest.mark.parametrize("attribute", ["port", "sysid", "body_name"])
def test_duplicate_json_route_values_are_rejected(attribute):
    first, second = DUAL_SITL_CHANNELS
    values = second.__dict__.copy()
    values[attribute] = getattr(first, attribute)
    with pytest.raises(ValueError, match=attribute):
        validate_channels((first, type(second)(**values)))


def test_dual_world_has_two_dynamic_free_bodies():
    world = SharedMuJoCoWorld()
    assert world.model.nq >= 14
    assert world.state("target").position.shape == (3,)
    assert world.state("chaser").position.shape == (3,)


def test_target_is_held_at_sky_pose_until_flight_release():
    world = SharedMuJoCoWorld()
    zero_thrust = (0.0, 0.0, 0.0, 0.0)
    initial = world.state("target")

    world.step({"target": zero_thrust, "chaser": zero_thrust}, substeps=250)

    held = world.state("target")
    assert held.time > initial.time
    assert not world.target_flight_released
    np.testing.assert_allclose(held.position, initial.position, atol=1e-9)
    np.testing.assert_allclose(held.velocity, (0.0, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(
        held.body_acceleration,
        (0.0, 0.0, abs(float(world.model.opt.gravity[2]))),
        atol=1e-9,
    )


def test_target_release_is_one_way_and_permits_physical_fall():
    world = SharedMuJoCoWorld()
    zero_thrust = (0.0, 0.0, 0.0, 0.0)
    initial_z = float(world.state("target").position[2])

    world.release_target_flight()
    world.release_target_flight()
    world.step({"target": zero_thrust, "chaser": zero_thrust}, substeps=100)

    released = world.state("target")
    assert world.target_flight_released
    assert released.position[2] < initial_z - 0.01
    assert released.velocity[2] < 0.0


def test_dynamic_target_geometries_remain_non_contacting():
    world = SharedMuJoCoWorld()
    target_body = mujoco.mj_name2id(
        world.model,
        mujoco.mjtObj.mjOBJ_BODY,
        "target",
    )
    target_geometries = np.flatnonzero(world.model.geom_bodyid == target_body)

    assert target_body >= 0
    assert target_geometries.size > 0
    assert np.all(world.model.geom_contype[target_geometries] == 0)
    assert np.all(world.model.geom_conaffinity[target_geometries] == 0)


def test_dual_world_camera_contract_uses_scenario_rate_when_enabled():
    world = SharedMuJoCoWorld()
    rate = world.scenario.two_drone_config.visualization.camera_rate_hz
    configs = camera_configs_for_scenario(
        "mountain_two_drones",
        camera_rate_hz=rate,
        values={"CAMERA_ENABLED": "1"},
    )
    assert len(configs) == 2
    assert {config.fps for config in configs} == {rate}


def test_compose_and_param_files_pin_distinct_vehicle_identity_contract():
    compose = (ROOT / "compose.yaml").read_text()
    assert 'profiles: ["dual-sitl"]' in compose
    assert '--sim-port-out=9003' in compose
    assert '--sim-port-out=9002' in compose
    for filename, sysid in (
        ("dual_sitl_target.parm", 2),
        ("dual_sitl_chaser.parm", 1),
    ):
        parameters = dict(
            line.split(maxsplit=1)
            for line in (ROOT / "config" / filename).read_text().splitlines()
            if line.strip()
        )
        assert parameters["SYSID_THISMAV"] == str(sysid)
        assert parameters["SIM_RATE_HZ"] == "250"
        assert parameters["SCHED_LOOP_RATE"] == "100"
        assert float(parameters["SIM_RATE_HZ"]) >= (
            1.8 * float(parameters["SCHED_LOOP_RATE"])
        )
