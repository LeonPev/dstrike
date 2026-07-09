from __future__ import annotations

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
