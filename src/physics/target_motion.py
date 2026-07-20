"""Deterministic terrain-parallel motion for the scripted target drone."""

from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np

from src.physics.scenario_config import TargetDroneConfig, heading_quaternion_wxyz
from src.physics.terrain_map import TerrainMap


@dataclass(frozen=True)
class TargetMotionState:
    time_s: float
    position_xyz: tuple[float, float, float]
    velocity_xyz: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    heading_deg: float
    distance_along_route_m: float
    direction: int


class TargetMotion:
    """Evaluate and apply the configured ping-pong route at any timestamp."""

    def __init__(self, config: TargetDroneConfig, terrain: TerrainMap) -> None:
        if config.route_behavior != "ping_pong":
            raise ValueError("target route_behavior must equal ping_pong")
        self.config = config
        self.terrain = terrain
        yaw = math.radians(config.normalized_heading_deg)
        self._unit_xy = (math.cos(yaw), math.sin(yaw))

    def state_at(self, time_s: float) -> TargetMotionState:
        if not math.isfinite(time_s) or time_s < 0.0:
            raise ValueError("target motion time must be finite and non-negative")
        length = self.config.route_length_m
        travelled = time_s * self.config.speed_mps
        cycle_distance = travelled % (2.0 * length)
        if cycle_distance < length:
            distance = cycle_distance
            direction = 1
        else:
            distance = 2.0 * length - cycle_distance
            direction = -1

        x_m = self.config.start_xy_m[0] + self._unit_xy[0] * distance
        y_m = self.config.start_xy_m[1] + self._unit_xy[1] * distance
        z_m = self.terrain.height_at(x_m, y_m) + self.config.altitude_agl_m
        heading = (self.config.normalized_heading_deg + (180.0 if direction < 0 else 0.0)) % 360.0
        horizontal_velocity = self.config.speed_mps * direction
        return TargetMotionState(
            time_s=float(time_s),
            position_xyz=(x_m, y_m, z_m),
            velocity_xyz=(
                self._unit_xy[0] * horizontal_velocity,
                self._unit_xy[1] * horizontal_velocity,
                0.0,
            ),
            quaternion_wxyz=heading_quaternion_wxyz(heading),
            heading_deg=heading,
            distance_along_route_m=distance,
            direction=direction,
        )

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData, time_s: float) -> TargetMotionState:
        """Set target mocap pose and refresh derived MuJoCo body transforms."""
        body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target"))
        if body_id < 0:
            raise ValueError("MuJoCo model must contain target body")
        mocap_id = int(model.body_mocapid[body_id])
        if mocap_id < 0:
            raise ValueError("target body must be a mocap body")
        state = self.state_at(time_s)
        data.mocap_pos[mocap_id] = np.asarray(state.position_xyz, dtype=float)
        data.mocap_quat[mocap_id] = np.asarray(state.quaternion_wxyz, dtype=float)
        mujoco.mj_forward(model, data)
        return state
