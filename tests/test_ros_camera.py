from __future__ import annotations

import math

from src.physics.ros_camera import (
    DEFAULT_CAMERA_INFO_TOPIC,
    DEFAULT_IMAGE_TOPIC,
    CameraConfig,
    camera_intrinsics_from_fovy,
    parse_bool,
)


def test_camera_config_defaults_match_dfd_011_topics() -> None:
    config = CameraConfig.from_mapping({})

    assert config.enabled is True
    assert config.camera_name == "front_cam"
    assert config.frame_id == "front_cam"
    assert config.image_topic == DEFAULT_IMAGE_TOPIC
    assert config.camera_info_topic == DEFAULT_CAMERA_INFO_TOPIC
    assert config.width == 640
    assert config.height == 480
    assert config.fps == 10.0


def test_camera_config_allows_camera_to_be_disabled() -> None:
    config = CameraConfig.from_mapping(
        {
            "CAMERA_ENABLED": "false",
            "CAMERA_WIDTH": "not-used-while-disabled",
            "CAMERA_FPS": "not-used-while-disabled",
        }
    )

    assert config.enabled is False


def test_parse_bool_rejects_ambiguous_values() -> None:
    try:
        parse_bool("maybe")
    except ValueError as exc:
        assert "expected boolean" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_camera_intrinsics_use_mujoco_vertical_fov() -> None:
    intrinsics = camera_intrinsics_from_fovy(width=640, height=480, fovy_degrees=90)

    assert math.isclose(intrinsics.fx, 240.0)
    assert math.isclose(intrinsics.fy, 240.0)
    assert intrinsics.cx == 319.5
    assert intrinsics.cy == 239.5
