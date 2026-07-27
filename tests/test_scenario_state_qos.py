from pathlib import Path


SOURCE = Path("src/ros2/drone_bridge/drone_bridge/scenario_controller.py").read_text()


def test_state_publisher_is_transient_local_and_periodic():
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in SOURCE
    assert 'self.create_timer(1.0, self.publish_state)' in SOURCE
    assert 'self.publish_state()' in SOURCE


def test_required_topics_and_entry_point_are_wired():
    for topic in (
        "/scenario/start", "/sim/target/odom", "/drone/odom",
        "/scenario/state", "/scenario/formation_error_m", "/drone/arm",
        "/drone/takeoff_alt_m", "/drone/cmd_vel",
    ):
        assert topic in SOURCE
    setup = Path("src/ros2/drone_bridge/setup.py").read_text()
    assert "scenario_controller = drone_bridge.scenario_controller:main" in setup


def test_arm_retry_is_rate_limited():
    assert "now_s - self.last_arm_request_s >= 1.0" in SOURCE
