from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from src.physics.scenarios import (
    DEFAULT_SCENARIO_NAME,
    build_scenario_model_xml,
    marker_specs_from_model_xml,
    scenario_from_mapping,
    scenario_from_name,
    scenario_marker_specs_from_xml,
)
from src.physics.scenario_config import heading_quaternion_wxyz
from src.physics.terrain_map import TerrainMap


BASE_MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "quad.xml"


def test_scenario_from_mapping_defaults_to_landing_pad() -> None:
    scenario = scenario_from_mapping({})

    assert scenario.name == DEFAULT_SCENARIO_NAME
    assert scenario.xml_path.name == "landing_pad.xml"


def test_default_alias_preserves_existing_env_files() -> None:
    scenario = scenario_from_name("default")

    assert scenario.name == "landing_pad"


def test_invalid_scenario_lists_available_names() -> None:
    try:
        scenario_from_name("maze")
    except ValueError as exc:
        assert "unknown scenario" in str(exc)
        assert "landing_pad" in str(exc)
        assert "obstacle_course" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_landing_pad_model_xml_overlays_scenario_geometry() -> None:
    model_xml = build_scenario_model_xml(
        BASE_MODEL_PATH,
        scenario_from_name("landing_pad"),
    )

    root = ET.fromstring(model_xml)
    geom_names = {
        geom.attrib["name"]
        for geom in root.findall("./worldbody/geom")
        if "name" in geom.attrib
    }

    assert "visual_ground" in geom_names
    assert "scenario_landing_pad" in geom_names
    assert "scenario_boundary_north" in geom_names
    assert "scenario_obstacle_1" not in geom_names


def test_landing_pad_markers_are_derived_from_xml_geometry() -> None:
    scenario = scenario_from_name("landing_pad")

    marker_by_namespace = {}
    for marker in scenario.marker_specs:
        marker_by_namespace.setdefault(marker.namespace, []).append(marker)

    assert marker_by_namespace["landing_pad"][0].shape == "cylinder"
    assert marker_by_namespace["landing_pad"][0].scale_xyz == (1.2, 1.2, 0.02)

    boundary_markers = marker_by_namespace["scenario_boundary"]
    assert len(boundary_markers) == 4
    assert {marker.shape for marker in boundary_markers} == {"cube"}
    assert {
        marker.position_xyz for marker in boundary_markers
    } == {
        (0.0, 5.0, 0.015),
        (0.0, -5.0, 0.015),
        (5.0, 0.0, 0.015),
        (-5.0, 0.0, 0.015),
    }
    assert (10.0, 0.05, 0.03) in {
        marker.scale_xyz for marker in boundary_markers
    }
    assert (0.05, 10.0, 0.03) in {
        marker.scale_xyz for marker in boundary_markers
    }


def test_obstacle_course_changes_model_and_marker_set() -> None:
    model_xml = build_scenario_model_xml(
        BASE_MODEL_PATH,
        scenario_from_name("obstacle_course"),
    )
    scenario = scenario_from_name("obstacle_course")

    root = ET.fromstring(model_xml)
    geom_names = {
        geom.attrib["name"]
        for geom in root.findall("./worldbody/geom")
        if "name" in geom.attrib
    }
    marker_namespaces = {marker.namespace for marker in scenario.marker_specs}
    markers_by_namespace = {}
    for marker in scenario.marker_specs:
        markers_by_namespace.setdefault(marker.namespace, []).append(marker)

    assert "scenario_obstacle_1" in geom_names
    assert "scenario_gate_crossbar" in geom_names
    assert marker_namespaces == {
        "landing_pad",
        "obstacles",
        "gates",
        "scenario_boundary",
    }
    assert len(markers_by_namespace["obstacles"]) == 2
    assert len(markers_by_namespace["gates"]) == 3
    assert len(markers_by_namespace["scenario_boundary"]) == 4
    assert {marker.shape for marker in scenario.marker_specs} == {"cube", "cylinder"}
    assert all(
        marker.shape == "cube"
        for marker in markers_by_namespace["scenario_boundary"]
    )


def test_xml_marker_generation_has_no_parallel_hardcoded_marker_shape() -> None:
    scenario = scenario_from_name("obstacle_course")
    direct_markers = scenario_marker_specs_from_xml(scenario.xml_path)

    assert scenario.marker_specs == direct_markers


def test_complete_model_markers_include_base_camera_target_visual() -> None:
    model_xml = build_scenario_model_xml(
        BASE_MODEL_PATH,
        scenario_from_name("obstacle_course"),
    )
    marker_by_namespace = {}
    for marker in marker_specs_from_model_xml(model_xml):
        marker_by_namespace.setdefault(marker.namespace, []).append(marker)

    camera_target = marker_by_namespace["visual_camera_target"][0]
    assert camera_target.shape == "cube"
    assert camera_target.position_xyz == (2.0, 0.0, 0.35)
    assert camera_target.scale_xyz == (0.36, 0.36, 0.7)
    assert camera_target.color_rgba == (0.95, 0.75, 0.10, 1.0)
    assert len(marker_by_namespace["scenario_boundary"]) == 4


def test_empty_world_has_no_scenario_markers() -> None:
    scenario = scenario_from_name("empty_world")

    assert scenario.marker_specs == ()


def test_mountain_map_uses_canonical_xml_lookup_and_strict_config() -> None:
    scenario = scenario_from_name("mountain-map")

    assert scenario.xml_path.name == "mountain_map.xml"
    assert scenario.xml_string == scenario.xml_path.read_text(encoding="utf-8")
    assert set(scenario.vfs_files) == {"mountain_map/heightfield.bin"}
    assert scenario.vfs_files["mountain_map/heightfield.bin"]
    assert scenario.config is not None
    assert scenario.config.name == "mountain_map"
    assert scenario.config.terrain.resolution == 129
    assert scenario.config.replaces_ground_plane is True
    assert scenario_marker_specs_from_xml(scenario.xml_path) == scenario.marker_specs


def test_mountain_map_patches_hfield_and_removes_flat_ground() -> None:
    scenario = scenario_from_name("mountain_map")
    model_xml = build_scenario_model_xml(
        BASE_MODEL_PATH,
        scenario,
    )
    root = ET.fromstring(model_xml)
    hfield = root.find("./asset/hfield[@name='mountain_hfield']")
    assert hfield is not None
    assert hfield.attrib["size"] == "150 150 45 3"
    geom_names = {
        geom.attrib.get("name") for geom in root.findall("./worldbody/geom")
    }
    assert "mountain_terrain" in geom_names
    assert "visual_ground" not in geom_names
    terrain_geom = root.find("./worldbody/geom[@name='mountain_terrain']")
    assert terrain_geom is not None
    assert terrain_geom.attrib["group"] == "2"

    launch_pad = root.find("./worldbody/geom[@name='scenario_landing_pad']")
    assert launch_pad is not None
    assert launch_pad.attrib["type"] == "cylinder"
    assert launch_pad.attrib["group"] == "1"
    pad_radius_m, pad_half_height_m = (
        float(value) for value in launch_pad.attrib["size"].split()
    )
    assert (pad_radius_m, pad_half_height_m) == (1.5, 0.05)
    assert scenario.config is not None
    terrain = TerrainMap.from_config(
        scenario.config,
        scenario.xml_path.parent / scenario.name,
    )
    terrain_height_m = terrain.height_at(
        scenario.config.spawn.x_m,
        scenario.config.spawn.y_m,
    )
    pad_position = tuple(float(value) for value in launch_pad.attrib["pos"].split())
    assert pad_position[:2] == (
        scenario.config.spawn.x_m,
        scenario.config.spawn.y_m,
    )
    assert math.isclose(
        pad_position[2] - pad_half_height_m,
        terrain_height_m,
        abs_tol=1e-6,
    )

    landing_pad_marker = next(
        marker
        for marker in marker_specs_from_model_xml(model_xml)
        if marker.namespace == "landing_pad"
    )
    assert landing_pad_marker.position_xyz == pad_position
    assert landing_pad_marker.scale_xyz == (3.0, 3.0, 0.1)
    assert landing_pad_marker.color_rgba == (0.95, 0.65, 0.08, 1.0)

    sun = root.find("./worldbody/light[@name='mountain_sun']")
    assert sun is not None
    assert sun.attrib["directional"] == "true"
    assert tuple(float(value) for value in sun.attrib["ambient"].split()) == (
        0.45,
        0.45,
        0.45,
    )
    sky = root.find("./asset/texture[@name='mountain_sky']")
    assert sky is not None
    assert sky.attrib["type"] == "skybox"

    visual_map = root.find("./visual/map")
    assert visual_map is not None
    assert float(visual_map.attrib["znear"]) == 0.0001


def test_heading_quaternion_contract() -> None:
    assert heading_quaternion_wxyz(0) == (1.0, 0.0, 0.0, 0.0)
    assert all(
        abs(actual - expected) < 1e-12
        for actual, expected in zip(
            heading_quaternion_wxyz(90),
            (2**-0.5, 0.0, 0.0, 2**-0.5),
        )
    )
    assert all(
        abs(actual - expected) < 1e-12
        for actual, expected in zip(
            heading_quaternion_wxyz(-90),
            (2**-0.5, 0.0, 0.0, -(2**-0.5)),
        )
    )
