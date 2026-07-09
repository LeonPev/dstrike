from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from src.physics.scenarios import (
    DEFAULT_SCENARIO_NAME,
    build_scenario_model_xml,
    scenario_from_mapping,
    scenario_from_name,
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

    assert "scenario_obstacle_1" in geom_names
    assert "scenario_gate_crossbar" in geom_names
    assert marker_namespaces == {
        "landing_pad",
        "obstacles",
        "gates",
        "scenario_boundary",
    }


def test_empty_world_has_no_scenario_markers() -> None:
    scenario = scenario_from_name("empty_world")

    assert scenario.marker_specs == ()
