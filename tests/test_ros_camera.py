from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from src.physics.ros_camera import (
    DEFAULT_CAMERA_INFO_TOPIC,
    DEFAULT_IMAGE_TOPIC,
    CameraConfig,
    camera_intrinsics_from_fovy,
    parse_bool,
    ros_image_pixels,
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


def test_front_camera_is_raised_and_pitched_down_for_terrain_view() -> None:
    root = ET.parse(Path(__file__).resolve().parents[1] / "models/quad.xml").getroot()
    camera = root.find("./worldbody/body[@name='base_link']/camera[@name='front_cam']")

    assert camera is not None
    assert tuple(float(value) for value in camera.attrib["pos"].split()) == (
        0.24,
        0.0,
        0.10,
    )
    axes = tuple(float(value) for value in camera.attrib["xyaxes"].split())
    assert axes[3] > 0.0
    assert axes[5] < 1.0


def test_ros_image_pixels_preserves_mujoco_top_left_row_order() -> None:
    pixels = np.array(
        [
            [[1, 2, 3], [4, 5, 6]],
            [[7, 8, 9], [10, 11, 12]],
        ],
        dtype=np.uint8,
    )

    oriented = ros_image_pixels(pixels)

    assert oriented.flags.c_contiguous
    assert oriented.tolist() == pixels.tolist()
