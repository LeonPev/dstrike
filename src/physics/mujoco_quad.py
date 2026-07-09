"""Minimal MuJoCo quadrotor wrapper for DFD-006."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import mujoco
import numpy as np


DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[2] / "models" / "quad.xml"
ROTOR_SITE_NAMES = (
    "rotor_front_left",
    "rotor_front_right",
    "rotor_rear_left",
    "rotor_rear_right",
)


@dataclass(frozen=True)
class QuadState:
    time: float
    position: np.ndarray
    quaternion: np.ndarray
    velocity: np.ndarray
    angular_velocity: np.ndarray
    body_acceleration: np.ndarray


class MuJoCoQuad:
    """Load and step the minimal free-body quadrotor model."""

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        model_xml: str | None = None,
        model_source: str | None = None,
    ) -> None:
        self.model_path = Path(model_path)
        self.model_source = model_source or str(self.model_path)
        if model_xml is None:
            self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        else:
            self.model = mujoco.MjModel.from_xml_string(model_xml)
        self.data = mujoco.MjData(self.model)

        self.body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )
        if self.body_id < 0:
            raise ValueError("MuJoCo model must contain body named base_link")

        self.rotor_site_ids = tuple(
            self._required_site_id(site_name) for site_name in ROTOR_SITE_NAMES
        )
        self.reset()

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def mass(self) -> float:
        return float(self.model.body_mass[self.body_id])

    def reset(
        self,
        position: Iterable[float] = (0.0, 0.0, 1.0),
        quaternion: Iterable[float] = (1.0, 0.0, 0.0, 0.0),
    ) -> QuadState:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = np.asarray(tuple(position), dtype=float)
        self.data.qpos[3:7] = _normalised_quaternion(quaternion)
        self.data.qvel[:] = 0.0
        self.data.qacc[:] = 0.0
        self.data.qfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def step(self, motor_thrusts: Iterable[float], substeps: int = 1) -> QuadState:
        thrusts = np.asarray(tuple(motor_thrusts), dtype=float)
        if thrusts.shape != (4,):
            raise ValueError("motor_thrusts must contain exactly four values")
        if np.any(thrusts < 0.0):
            raise ValueError("motor_thrusts must be non-negative")
        if substeps < 1:
            raise ValueError("substeps must be at least 1")

        for _ in range(substeps):
            self.data.qfrc_applied[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            body_rotation = self.data.xmat[self.body_id].reshape(3, 3)
            thrust_axis_world = body_rotation @ np.array([0.0, 0.0, 1.0])

            for site_id, thrust in zip(self.rotor_site_ids, thrusts):
                force_world = thrust_axis_world * thrust
                point_world = self.data.site_xpos[site_id]
                mujoco.mj_applyFT(
                    self.model,
                    self.data,
                    force_world,
                    np.zeros(3),
                    point_world,
                    self.body_id,
                    self.data.qfrc_applied,
                )

            mujoco.mj_step(self.model, self.data)

        return self.state()

    def state(self) -> QuadState:
        body_rotation = self.data.xmat[self.body_id].reshape(3, 3)
        body_acceleration = body_rotation.T @ self.data.qacc[0:3]
        return QuadState(
            time=float(self.data.time),
            position=np.array(self.data.qpos[0:3], dtype=float),
            quaternion=np.array(self.data.qpos[3:7], dtype=float),
            velocity=np.array(self.data.qvel[0:3], dtype=float),
            angular_velocity=np.array(self.data.qvel[3:6], dtype=float),
            body_acceleration=np.array(body_acceleration, dtype=float),
        )

    def _required_site_id(self, site_name: str) -> int:
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id < 0:
            raise ValueError(f"MuJoCo model must contain site named {site_name}")
        return site_id


def _normalised_quaternion(values: Iterable[float]) -> np.ndarray:
    quaternion = np.asarray(tuple(values), dtype=float)
    if quaternion.shape != (4,):
        raise ValueError("quaternion must contain exactly four values")
    norm = np.linalg.norm(quaternion)
    if norm <= 0.0:
        raise ValueError("quaternion must have non-zero length")
    return quaternion / norm
