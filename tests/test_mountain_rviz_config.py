from __future__ import annotations

from pathlib import Path

import yaml


def test_mountain_map_rviz_keeps_live_front_camera_display() -> None:
    config_path = Path(__file__).resolve().parents[1] / "config/mountain_map.rviz"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]

    camera = next(
        display
        for display in displays
        if display.get("Class") == "rviz_default_plugins/Image"
    )
    assert camera["Enabled"] is True
    assert camera["Name"] == "Front Camera"
    assert camera["Topic"]["Value"] == "/sim/camera/front/image_raw"
