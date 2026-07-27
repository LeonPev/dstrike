from pathlib import Path
import sys

import pytest

from src.physics.multi_vehicle_json import DUAL_SITL_CHANNELS, SharedMuJoCoWorld, validate_channels


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


def test_compose_and_param_files_pin_distinct_vehicle_identity_contract():
    compose = (ROOT / "compose.yaml").read_text()
    assert 'profiles: ["dual-sitl"]' in compose
    assert '--sim-port-out=9003' in compose
    assert '--sim-port-out=9002' in compose
    assert (ROOT / "config/dual_sitl_target.parm").read_text().splitlines()[0] == "SYSID_THISMAV 2"
    assert (ROOT / "config/dual_sitl_chaser.parm").read_text().splitlines()[0] == "SYSID_THISMAV 1"
