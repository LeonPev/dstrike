"""Compose the SCN-002 target and chaser into the mountain MuJoCo world."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from src.physics.scenario_config import TwoDroneScenarioConfig, heading_quaternion_wxyz
from src.physics.terrain_map import TerrainMap


PROJECT_ROOT = Path(__file__).resolve().parents[2]
QUAD_VISUAL_PATH = PROJECT_ROOT / "models/common/quad_visual.xml"
CHASER_BODY_NAME = "chaser"
TARGET_BODY_NAME = "target"


def compose_two_drone_world(
    base_root: ET.Element,
    scenario_root: ET.Element,
    config: TwoDroneScenarioConfig,
    terrain: TerrainMap,
) -> None:
    """Patch validated spawn poses and shared quad geometry into both bodies."""
    base_body = base_root.find("./worldbody/body[@name='base_link']")
    if base_body is None:
        raise ValueError("base model must define the ArduPilot base_link body")
    target_body = scenario_root.find("./worldbody/body[@name='target']")
    if target_body is None:
        raise ValueError("mountain_two_drones XML must define target body")

    base_body.set("name", CHASER_BODY_NAME)
    freejoint = base_body.find("freejoint")
    if freejoint is None:
        raise ValueError("chaser must retain the ArduPilot free joint")
    freejoint.set("name", "chaser_free_joint")

    for child in list(base_body):
        if child.tag in {"geom", "site", "camera"}:
            base_body.remove(child)
    _append_quad_visual(base_body, "chaser", contacting=True)
    _append_quad_visual(target_body, "target", contacting=False)

    chaser = config.chaser_drone
    target = config.target_drone
    _set_pose(
        base_body,
        chaser.start_xy_m,
        terrain.height_at(*chaser.start_xy_m) + chaser.initial_agl_m,
        chaser.heading_deg,
    )
    _set_pose(
        target_body,
        target.start_xy_m,
        terrain.height_at(*target.start_xy_m) + target.altitude_agl_m,
        target.heading_deg,
    )


def _append_quad_visual(body: ET.Element, prefix: str, *, contacting: bool) -> None:
    template = ET.parse(QUAD_VISUAL_PATH).getroot().find("./worldbody/body")
    if template is None:
        raise ValueError("shared quad visual must define a template body")
    contact_value = "1" if contacting else "0"
    for template_child in list(template):
        child = deepcopy(template_child)
        if "name" in child.attrib:
            child.set("name", child.attrib["name"].format(prefix=prefix))
        if child.tag == "geom":
            child.set("contype", contact_value)
            child.set("conaffinity", contact_value)
        body.append(child)


def _set_pose(
    body: ET.Element,
    xy_m: tuple[float, float],
    z_m: float,
    heading_deg: float,
) -> None:
    body.set("pos", f"{xy_m[0]:.9g} {xy_m[1]:.9g} {z_m:.9g}")
    body.set(
        "quat",
        " ".join(f"{value:.12g}" for value in heading_quaternion_wxyz(heading_deg)),
    )
