"""Scenario selection and MuJoCo-backed RViz marker metadata."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from typing import Mapping

from src.physics.scenario_config import MountainMapConfig, load_mountain_map_config
from src.physics.terrain_map import TerrainMap


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCENARIOS_DIR = PROJECT_ROOT / "models" / "scenarios"
DEFAULT_SCENARIO_NAME = "landing_pad"
SCENARIO_ENV_VAR = "SCENARIO"
MOUNTAIN_LAUNCH_PAD_GEOM_NAME = "scenario_landing_pad"


@dataclass(frozen=True)
class ScenarioMarkerSpec:
    namespace: str
    marker_id: int
    shape: str
    position_xyz: tuple[float, float, float]
    scale_xyz: tuple[float, float, float]
    color_rgba: tuple[float, float, float, float]
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    points_xyz: tuple[tuple[float, float, float], ...] = ()


@dataclass(frozen=True)
class ScenarioConfig:
    name: str
    xml_path: Path
    marker_specs: tuple[ScenarioMarkerSpec, ...]
    xml_string: str
    vfs_files: Mapping[str, bytes]
    config: MountainMapConfig | None


def scenario_from_env() -> ScenarioConfig:
    return scenario_from_mapping(os.environ)


def scenario_from_mapping(values: Mapping[str, str]) -> ScenarioConfig:
    return scenario_from_name(values.get(SCENARIO_ENV_VAR, DEFAULT_SCENARIO_NAME))


def scenario_from_name(name: str | None) -> ScenarioConfig:
    normalized_name = normalize_scenario_name(name)
    if normalized_name == "default":
        normalized_name = DEFAULT_SCENARIO_NAME

    xml_path = SCENARIOS_DIR / f"{normalized_name}.xml"
    if not xml_path.is_file():
        allowed = ", ".join(
            sorted(
                scenario_xml_path.stem
                for scenario_xml_path in SCENARIOS_DIR.glob("*.xml")
            )
        )
        raise ValueError(
            f"unknown scenario {name!r}; expected one of: {allowed}"
        )

    config_path = SCENARIOS_DIR / normalized_name / "scenario.yaml"
    config = (
        load_mountain_map_config(config_path)
        if normalized_name == "mountain_map"
        else None
    )
    xml_string = xml_path.read_text(encoding="utf-8")
    vfs_files: dict[str, bytes] = {}
    if config is not None:
        asset_path = SCENARIOS_DIR / normalized_name / "heightfield.bin"
        height_map_path = SCENARIOS_DIR / normalized_name / "heightfield_m.npy"
        if not asset_path.is_file() or not height_map_path.is_file():
            raise FileNotFoundError(
                "mountain_map assets missing; run `make generate-map`"
            )
        vfs_files["mountain_map/heightfield.bin"] = asset_path.read_bytes()

    marker_root = ET.fromstring(xml_string)
    if config is not None:
        _patch_mountain_scenario_root(marker_root, config, config_path.parent)
    return ScenarioConfig(
        name=normalized_name,
        xml_path=xml_path,
        marker_specs=_marker_specs_from_root(marker_root, ("scenario_",)),
        xml_string=xml_string,
        vfs_files=vfs_files,
        config=config,
    )


def normalize_scenario_name(name: str | None) -> str:
    if name is None:
        return DEFAULT_SCENARIO_NAME
    normalized = name.strip().lower()
    if not normalized:
        return DEFAULT_SCENARIO_NAME
    return normalized.replace("-", "_")


def build_scenario_model_xml(
    base_model_path: str | Path,
    scenario: ScenarioConfig,
) -> str:
    """Merge a scenario fragment into the base MuJoCo model XML."""
    base_path = Path(base_model_path)
    if not base_path.is_file():
        raise FileNotFoundError(f"base MuJoCo model not found: {base_path}")
    if not scenario.xml_path.is_file():
        raise FileNotFoundError(f"scenario XML not found: {scenario.xml_path}")

    base_root = ET.parse(base_path).getroot()
    scenario_root = ET.fromstring(scenario.xml_string)

    if scenario.config is not None:
        _prepare_mountain_map_xml(base_root, scenario_root, scenario)

    _merge_section_children(base_root, scenario_root, "asset")
    _merge_section_children(base_root, scenario_root, "visual")
    _merge_section_children(base_root, scenario_root, "worldbody")
    return ET.tostring(base_root, encoding="unicode")


def _prepare_mountain_map_xml(
    base_root: ET.Element,
    scenario_root: ET.Element,
    scenario: ScenarioConfig,
) -> None:
    """Patch config-owned hfield dimensions and remove the flat base plane."""
    config = scenario.config
    if config is None:
        raise ValueError("mountain_map scenario config is required")
    _patch_mountain_scenario_root(
        scenario_root,
        config,
        scenario.xml_path.parent / scenario.name,
    )

    worldbody = base_root.find("worldbody")
    if worldbody is not None:
        for geom in list(worldbody.findall("geom")):
            if geom.attrib.get("name") == "visual_ground":
                worldbody.remove(geom)


def _patch_mountain_scenario_root(
    scenario_root: ET.Element,
    config: MountainMapConfig,
    asset_dir: Path,
) -> None:
    """Patch the heightfield and finite launch pad from canonical map data."""
    hfield = scenario_root.find("./asset/hfield[@name='mountain_hfield']")
    if hfield is None:
        raise ValueError("mountain_map XML must define mountain_hfield")
    terrain = config.terrain
    hfield.set(
        "size",
        " ".join(
            f"{value:g}"
            for value in (
                terrain.size_m / 2.0,
                terrain.size_m / 2.0,
                terrain.max_height_m,
                terrain.base_depth_m,
            )
        ),
    )

    launch_pad = scenario_root.find(
        f"./worldbody/geom[@name='{MOUNTAIN_LAUNCH_PAD_GEOM_NAME}']"
    )
    if launch_pad is None:
        raise ValueError(
            f"mountain_map XML must define {MOUNTAIN_LAUNCH_PAD_GEOM_NAME}"
        )
    if launch_pad.attrib.get("type") != "cylinder":
        raise ValueError("mountain launch pad must be a cylinder")
    radius_m, half_height_m = _parse_float_tuple(
        launch_pad.attrib.get("size", ""),
        2,
        MOUNTAIN_LAUNCH_PAD_GEOM_NAME,
    )
    if not all(
        math.isfinite(value) and value > 0.0
        for value in (radius_m, half_height_m)
    ):
        raise ValueError("mountain launch pad size values must be positive and finite")

    terrain = TerrainMap.from_config(config, asset_dir)
    terrain_height_m = terrain.height_at(config.spawn.x_m, config.spawn.y_m)
    launch_pad.set(
        "pos",
        " ".join(
            f"{value:.9g}"
            for value in (
                config.spawn.x_m,
                config.spawn.y_m,
                terrain_height_m + half_height_m,
            )
        ),
    )


def mountain_launch_pad_top_z(scenario: ScenarioConfig) -> float:
    """Return the configured mountain launch surface height in world z-up."""
    if scenario.config is None:
        raise ValueError("mountain launch pad requires a configured map scenario")
    pads = [
        marker
        for marker in scenario.marker_specs
        if marker.namespace == "landing_pad"
    ]
    if len(pads) != 1:
        raise ValueError("mountain_map must define exactly one landing pad")
    pad = pads[0]
    top_z = pad.position_xyz[2] + pad.scale_xyz[2] / 2.0
    if not math.isfinite(top_z):
        raise ValueError("mountain launch pad top must be finite")
    return top_z


def scenario_marker_specs_from_xml(
    xml_path: str | Path,
) -> tuple[ScenarioMarkerSpec, ...]:
    """Build RViz markers from the same MuJoCo scenario geoms used by physics."""
    path = Path(xml_path)
    root = ET.parse(path).getroot()
    if path.stem == "mountain_map":
        config_path = path.parent / path.stem / "scenario.yaml"
        if config_path.is_file():
            config = load_mountain_map_config(config_path)
            _patch_mountain_scenario_root(root, config, config_path.parent)
    return _marker_specs_from_root(root, ("scenario_",))


def marker_specs_from_model_xml(model_xml: str) -> tuple[ScenarioMarkerSpec, ...]:
    """Build RViz markers from the complete MuJoCo model XML."""
    return _marker_specs_from_root(ET.fromstring(model_xml), ("scenario_", "visual_"))


def _marker_specs_from_root(
    root: ET.Element,
    marker_prefixes: tuple[str, ...],
) -> tuple[ScenarioMarkerSpec, ...]:
    material_colors = _material_colors(root)
    marker_specs: list[ScenarioMarkerSpec] = []
    namespace_counts: dict[str, int] = {}

    for geom in root.findall("./worldbody/geom"):
        geom_name = geom.attrib.get("name", "")
        geom_type = geom.attrib.get("type", "")
        if not geom_name.startswith(marker_prefixes):
            continue

        if geom_type == "box":
            shape = "cube"
            size = _parse_float_tuple(geom.attrib.get("size", ""), 3, geom_name)
            scale_xyz = (2.0 * size[0], 2.0 * size[1], 2.0 * size[2])
        elif geom_type == "cylinder":
            shape = "cylinder"
            size = _parse_float_tuple(geom.attrib.get("size", ""), 2, geom_name)
            scale_xyz = (2.0 * size[0], 2.0 * size[0], 2.0 * size[1])
        else:
            continue

        namespace = _namespace_from_geom_name(geom_name)
        marker_id = namespace_counts.get(namespace, 0)
        namespace_counts[namespace] = marker_id + 1
        marker_specs.append(
            ScenarioMarkerSpec(
                namespace=namespace,
                marker_id=marker_id,
                shape=shape,
                position_xyz=_parse_float_tuple(
                    geom.attrib.get("pos", "0 0 0"),
                    3,
                    geom_name,
                ),
                scale_xyz=scale_xyz,
                color_rgba=_geom_color_rgba(geom, material_colors),
            )
        )

    return tuple(marker_specs)


def _merge_section_children(
    base_root: ET.Element,
    scenario_root: ET.Element,
    section_name: str,
) -> None:
    base_section = base_root.find(section_name)
    if base_section is None:
        base_section = ET.SubElement(base_root, section_name)

    for scenario_section in scenario_root.findall(section_name):
        for child in list(scenario_section):
            base_section.append(child)


def _material_colors(root: ET.Element) -> dict[str, tuple[float, float, float, float]]:
    colors: dict[str, tuple[float, float, float, float]] = {}
    for material in root.findall("./asset/material"):
        name = material.attrib.get("name")
        rgba = material.attrib.get("rgba")
        if name and rgba:
            colors[name] = _parse_float_tuple(rgba, 4, name)
    return colors


def _geom_color_rgba(
    geom: ET.Element,
    material_colors: Mapping[str, tuple[float, float, float, float]],
) -> tuple[float, float, float, float]:
    if "rgba" in geom.attrib:
        return _parse_float_tuple(
            geom.attrib["rgba"],
            4,
            geom.attrib.get("name", "geom"),
        )
    material_name = geom.attrib.get("material")
    if material_name and material_name in material_colors:
        return material_colors[material_name]
    return (0.8, 0.8, 0.8, 1.0)


def _namespace_from_geom_name(geom_name: str) -> str:
    if geom_name == "visual_camera_target":
        return "visual_camera_target"
    if geom_name == "scenario_landing_pad":
        return "landing_pad"
    if geom_name.startswith("scenario_obstacle_"):
        return "obstacles"
    if geom_name.startswith("scenario_gate_"):
        return "gates"
    if geom_name.startswith("scenario_boundary_"):
        return "scenario_boundary"
    return geom_name.removeprefix("scenario_")


def _parse_float_tuple(
    text: str,
    expected_count: int,
    context: str,
) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split())
    if len(values) != expected_count:
        raise ValueError(
            f"{context} expected {expected_count} float values, got {len(values)}"
        )
    return values
