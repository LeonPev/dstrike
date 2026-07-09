"""Scenario selection, MuJoCo XML overlays, and RViz marker metadata."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from typing import Iterable, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCENARIOS_DIR = PROJECT_ROOT / "models" / "scenarios"
DEFAULT_SCENARIO_NAME = "landing_pad"
SCENARIO_ENV_VAR = "SCENARIO"


@dataclass(frozen=True)
class ScenarioMarkerSpec:
    namespace: str
    marker_id: int
    shape: str
    position_xyz: tuple[float, float, float]
    scale_xyz: tuple[float, float, float]
    color_rgba: tuple[float, float, float, float]
    points_xyz: tuple[tuple[float, float, float], ...] = ()


@dataclass(frozen=True)
class ScenarioConfig:
    name: str
    xml_path: Path
    marker_specs: tuple[ScenarioMarkerSpec, ...]


def scenario_from_env() -> ScenarioConfig:
    return scenario_from_mapping(os.environ)


def scenario_from_mapping(values: Mapping[str, str]) -> ScenarioConfig:
    return scenario_from_name(values.get(SCENARIO_ENV_VAR, DEFAULT_SCENARIO_NAME))


def scenario_from_name(name: str | None) -> ScenarioConfig:
    normalized_name = normalize_scenario_name(name)
    if normalized_name == "default":
        normalized_name = DEFAULT_SCENARIO_NAME

    try:
        marker_specs = _SCENARIO_MARKERS[normalized_name]
    except KeyError as exc:
        allowed = ", ".join(sorted(_SCENARIO_MARKERS))
        raise ValueError(
            f"unknown scenario {name!r}; expected one of: {allowed}"
        ) from exc

    return ScenarioConfig(
        name=normalized_name,
        xml_path=SCENARIOS_DIR / f"{normalized_name}.xml",
        marker_specs=marker_specs,
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
    scenario_root = ET.parse(scenario.xml_path).getroot()

    _merge_section_children(base_root, scenario_root, "asset")
    _merge_section_children(base_root, scenario_root, "worldbody")
    return ET.tostring(base_root, encoding="unicode")


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


def _marker(
    namespace: str,
    marker_id: int,
    shape: str,
    position_xyz: Iterable[float],
    scale_xyz: Iterable[float],
    color_rgba: Iterable[float],
    points_xyz: Iterable[Iterable[float]] = (),
) -> ScenarioMarkerSpec:
    return ScenarioMarkerSpec(
        namespace=namespace,
        marker_id=marker_id,
        shape=shape,
        position_xyz=_tuple3(position_xyz),
        scale_xyz=_tuple3(scale_xyz),
        color_rgba=_tuple4(color_rgba),
        points_xyz=tuple(_tuple3(point) for point in points_xyz),
    )


def _tuple3(values: Iterable[float]) -> tuple[float, float, float]:
    value_tuple = tuple(float(value) for value in values)
    if len(value_tuple) != 3:
        raise ValueError("expected exactly three values")
    return value_tuple


def _tuple4(values: Iterable[float]) -> tuple[float, float, float, float]:
    value_tuple = tuple(float(value) for value in values)
    if len(value_tuple) != 4:
        raise ValueError("expected exactly four values")
    return value_tuple


LANDING_PAD_MARKER = _marker(
    namespace="landing_pad",
    marker_id=0,
    shape="cylinder",
    position_xyz=(0.0, 0.0, 0.01),
    scale_xyz=(1.2, 1.2, 0.02),
    color_rgba=(0.1, 0.8, 0.35, 0.8),
)
BOUNDARY_MARKER = _marker(
    namespace="scenario_boundary",
    marker_id=0,
    shape="line_strip",
    position_xyz=(0.0, 0.0, 0.0),
    scale_xyz=(0.03, 0.0, 0.0),
    color_rgba=(1.0, 0.85, 0.05, 0.9),
    points_xyz=(
        (-5.0, -5.0, 0.02),
        (5.0, -5.0, 0.02),
        (5.0, 5.0, 0.02),
        (-5.0, 5.0, 0.02),
        (-5.0, -5.0, 0.02),
    ),
)
OBSTACLE_MARKERS = (
    _marker(
        namespace="obstacles",
        marker_id=1,
        shape="cube",
        position_xyz=(1.8, -0.8, 0.5),
        scale_xyz=(0.45, 0.45, 1.0),
        color_rgba=(0.9, 0.2, 0.1, 0.75),
    ),
    _marker(
        namespace="obstacles",
        marker_id=2,
        shape="cube",
        position_xyz=(3.6, 1.1, 0.5),
        scale_xyz=(0.45, 0.45, 1.0),
        color_rgba=(0.9, 0.2, 0.1, 0.75),
    ),
)
GATE_MARKER = _marker(
    namespace="gates",
    marker_id=0,
    shape="line_list",
    position_xyz=(0.0, 0.0, 0.0),
    scale_xyz=(0.04, 0.0, 0.0),
    color_rgba=(0.15, 0.45, 1.0, 0.9),
    points_xyz=(
        (2.8, -0.8, 0.0),
        (2.8, -0.8, 1.6),
        (2.8, 0.8, 0.0),
        (2.8, 0.8, 1.6),
        (2.8, -0.8, 1.6),
        (2.8, 0.8, 1.6),
    ),
)


_SCENARIO_MARKERS: dict[str, tuple[ScenarioMarkerSpec, ...]] = {
    "empty_world": (),
    "landing_pad": (LANDING_PAD_MARKER, BOUNDARY_MARKER),
    "obstacle_course": (
        LANDING_PAD_MARKER,
        *OBSTACLE_MARKERS,
        GATE_MARKER,
        BOUNDARY_MARKER,
    ),
}
