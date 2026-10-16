from __future__ import annotations

from pathlib import Path

import mujoco

from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.ros_camera import (
    CHASER_CAMERA_FRAME,
    CHASER_CAMERA_INFO_TOPIC,
    CHASER_CAMERA_NAME,
    CHASER_IMAGE_TOPIC,
    DEFAULT_CAMERA_INFO_TOPIC,
    DEFAULT_CAMERA_NAME,
    DEFAULT_IMAGE_TOPIC,
    TARGET_CAMERA_FRAME,
    TARGET_CAMERA_INFO_TOPIC,
    TARGET_CAMERA_NAME,
    TARGET_IMAGE_TOPIC,
    camera_configs_for_scenario,
)
from src.physics.scenarios import build_scenario_model_xml, scenario_from_name


ROOT = Path(__file__).resolve().parents[1]


def test_two_drone_camera_contract_uses_config_rate_and_namespaces() -> None:
    configs = camera_configs_for_scenario(
        "mountain_two_drones",
        camera_rate_hz=5.0,
        values={"CAMERA_ENABLED": "1", "CAMERA_WIDTH": "320", "CAMERA_HEIGHT": "240"},
    )

    assert [config.camera_name for config in configs] == [
        TARGET_CAMERA_NAME,
        CHASER_CAMERA_NAME,
    ]
    assert [config.frame_id for config in configs] == [
        TARGET_CAMERA_FRAME,
        CHASER_CAMERA_FRAME,
    ]
    assert [config.image_topic for config in configs] == [
        TARGET_IMAGE_TOPIC,
        CHASER_IMAGE_TOPIC,
    ]
    assert [config.camera_info_topic for config in configs] == [
        TARGET_CAMERA_INFO_TOPIC,
        CHASER_CAMERA_INFO_TOPIC,
    ]
    assert all(config.fps == 5.0 for config in configs)
    assert all((config.width, config.height) == (320, 240) for config in configs)


def test_camera_enabled_switch_disables_all_two_drone_streams() -> None:
    assert camera_configs_for_scenario(
        "mountain_two_drones",
        camera_rate_hz=5.0,
        values={"CAMERA_ENABLED": "0"},
    ) == ()


def test_non_two_drone_scenario_preserves_single_camera_contract() -> None:
    (config,) = camera_configs_for_scenario(
        "mountain_map",
        values={"CAMERA_ENABLED": "1"},
    )

    assert config.camera_name == DEFAULT_CAMERA_NAME
    assert config.image_topic == DEFAULT_IMAGE_TOPIC
    assert config.camera_info_topic == DEFAULT_CAMERA_INFO_TOPIC


def test_compiled_two_drone_model_has_both_front_cameras() -> None:
    scenario = scenario_from_name("mountain_two_drones")
    quad = MuJoCoQuad(
        model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario),
        vfs_files=scenario.vfs_files,
    )

    for name in (TARGET_CAMERA_NAME, CHASER_CAMERA_NAME):
        assert mujoco.mj_name2id(quad.model, mujoco.mjtObj.mjOBJ_CAMERA, name) >= 0


def test_camera_resolution_defaults_remain_bounded_for_docker_rendering() -> None:
    configs = camera_configs_for_scenario(
        "mountain_two_drones",
        camera_rate_hz=5.0,
        values={"CAMERA_ENABLED": "1"},
    )

    assert all(config.width * config.height <= 640 * 480 for config in configs)
