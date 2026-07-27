"""Shared-world JSON physics server for the optional SCN-010 dual SITL profile."""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
import selectors
import socket
import sys
from typing import Iterable, TextIO

import mujoco
import numpy as np

from src.physics.ardupilot_json_stub import PacketError, parse_pwm_packet
from src.physics.frame_conventions import ardupilot_state_from_quad_state
from src.physics.motor_model import ARDUPILOT_QUAD_X_ROTORS, DEFAULT_YAW_MOMENT_ARM_M
from src.physics.mujoco_json import (
    DEFAULT_MAX_FRAME_DT,
    DEFAULT_PWM_MAX,
    DEFAULT_PWM_MIN,
    DEFAULT_THRUST_TO_WEIGHT,
    FixedStepClock,
    GRAVITY_MPS2,
    PwmThrustMapper,
)
from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, QuadState
from src.physics.ros_camera import SimCameraPublishers, camera_configs_for_scenario
from src.physics.ros_visualization import SimVisualizationPublisher
from src.physics.scenarios import build_scenario_model_xml, marker_specs_from_model_xml, scenario_from_name
from src.physics.terrain_map import TerrainMap
from src.physics.two_drone_world import chaser_launch_surface_z


DEFAULT_CHANNELS = {"target": 9003, "chaser": 9002}


@dataclass(frozen=True)
class VehicleChannel:
    role: str
    port: int
    sysid: int
    body_name: str


DUAL_SITL_CHANNELS = (
    VehicleChannel("target", 9003, 2, "target"),
    VehicleChannel("chaser", 9002, 1, "chaser"),
)


def validate_channels(channels: Iterable[VehicleChannel]) -> tuple[VehicleChannel, ...]:
    result = tuple(channels)
    if {item.role for item in result} != {"target", "chaser"}:
        raise ValueError("dual SITL requires exactly target and chaser channels")
    for field in ("port", "sysid", "body_name"):
        values = [getattr(item, field) for item in result]
        if len(values) != len(set(values)):
            raise ValueError(f"dual SITL {field} values must be unique")
    return result


class SharedMuJoCoWorld:
    """Apply two autopilot output streams to distinct free bodies in one model."""

    def __init__(self) -> None:
        scenario = scenario_from_name("mountain_two_drones")
        if scenario.config is None or scenario.two_drone_config is None:
            raise ValueError("dual SITL requires the mountain_two_drones scenario")
        terrain = TerrainMap.from_config(scenario.config, scenario.xml_path.parent / "mountain_map")
        xml = build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario, dynamic_target=True)
        vfs = mujoco.MjVfs()
        for filename, contents in scenario.vfs_files.items():
            vfs[filename] = contents
        self.model = mujoco.MjModel.from_xml_string(xml, vfs=vfs)
        self.data = mujoco.MjData(self.model)
        self.scenario = scenario
        self.terrain = terrain
        self.model_xml = xml
        self._body_ids = {role: self._id(mujoco.mjtObj.mjOBJ_BODY, role) for role in DEFAULT_CHANNELS}
        self._joint_ids = {role: self._id(mujoco.mjtObj.mjOBJ_JOINT, f"{role}_free_joint") for role in DEFAULT_CHANNELS}
        self._site_ids = {
            role: tuple(self._id(mujoco.mjtObj.mjOBJ_SITE, f"{role}_{rotor.site_suffix}") for rotor in ARDUPILOT_QUAD_X_ROTORS)
            for role in DEFAULT_CHANNELS
        }
        self.rotor_site_ids = tuple(self._id(mujoco.mjtObj.mjOBJ_SITE, f"chaser_rotor_{name}") for name in ("front_left", "front_right", "rear_left", "rear_right"))
        self.body_id = self._body_ids["chaser"]
        self.reset()

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def _id(self, kind: mujoco.mjtObj, name: str) -> int:
        value = mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise ValueError(f"dual SITL model is missing {name}")
        return value

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        config = self.scenario.two_drone_config
        assert config is not None
        poses = {
            "chaser": (
                config.chaser_drone.start_xy_m,
                chaser_launch_surface_z(config, self.terrain)
                + config.chaser_drone.initial_agl_m,
            ),
            "target": (
                config.target_drone.start_xy_m,
                self.terrain.height_at(*config.target_drone.start_xy_m)
                + config.target_drone.altitude_agl_m,
            ),
        }
        from src.physics.scenario_config import heading_quaternion_wxyz
        for role, (xy, z) in poses.items():
            joint = self._joint_ids[role]
            qpos = int(self.model.jnt_qposadr[joint])
            self.data.qpos[qpos:qpos + 3] = (xy[0], xy[1], z)
            heading = config.chaser_drone.heading_deg if role == "chaser" else config.target_drone.heading_deg
            self.data.qpos[qpos + 3:qpos + 7] = heading_quaternion_wxyz(heading)
        mujoco.mj_forward(self.model, self.data)

    def mass(self, role: str) -> float:
        return float(self.model.body_mass[self._body_ids[role]])

    def state(self, role: str = "chaser") -> QuadState:
        body = self._body_ids[role]
        joint = self._joint_ids[role]
        qpos = int(self.model.jnt_qposadr[joint])
        dof = int(self.model.jnt_dofadr[joint])
        rotation = self.data.xmat[body].reshape(3, 3)
        specific_force = self.data.qacc[dof:dof + 3] - self.model.opt.gravity
        return QuadState(float(self.data.time), np.array(self.data.qpos[qpos:qpos + 3]), np.array(self.data.qpos[qpos + 3:qpos + 7]), np.array(self.data.qvel[dof:dof + 3]), np.array(self.data.qvel[dof + 3:dof + 6]), np.array(rotation.T @ specific_force))

    def step(self, thrust_by_role: dict[str, tuple[float, float, float, float]], substeps: int) -> None:
        for _ in range(substeps):
            self.data.qfrc_applied[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            for role, thrusts in thrust_by_role.items():
                body = self._body_ids[role]
                axis = self.data.xmat[body].reshape(3, 3) @ np.array((0.0, 0.0, 1.0))
                for rotor, site, thrust in zip(ARDUPILOT_QUAD_X_ROTORS, self._site_ids[role], thrusts):
                    mujoco.mj_applyFT(self.model, self.data, axis * thrust, axis * rotor.yaw_torque_sign * thrust * DEFAULT_YAW_MOMENT_ARM_M, self.data.site_xpos[site], body, self.data.qfrc_applied)
            mujoco.mj_step(self.model, self.data)


def serve(host: str = "0.0.0.0", stdout: TextIO = sys.stdout, stderr: TextIO = sys.stderr) -> None:
    channels = validate_channels(DUAL_SITL_CHANNELS)
    world = SharedMuJoCoWorld()
    mappers = {item.role: PwmThrustMapper(DEFAULT_PWM_MIN, DEFAULT_PWM_MAX, world.mass(item.role) * GRAVITY_MPS2 * DEFAULT_THRUST_TO_WEIGHT / 4.0) for item in channels}
    clocks = {item.role: FixedStepClock(world.timestep, DEFAULT_MAX_FRAME_DT) for item in channels}
    thrusts = {item.role: (0.0, 0.0, 0.0, 0.0) for item in channels}
    selector = selectors.DefaultSelector()
    sockets: dict[socket.socket, VehicleChannel] = {}
    visualization = SimVisualizationPublisher(world, publish_rate_hz=10.0, scenario_markers=marker_specs_from_model_xml(world.model_xml), publish_target=True, two_drone_config=world.scenario.two_drone_config, terrain=world.terrain)
    camera_configs = camera_configs_for_scenario(
        "mountain_two_drones",
        camera_rate_hz=world.scenario.two_drone_config.visualization.camera_rate_hz,
    )
    cameras = SimCameraPublishers(world, camera_configs) if camera_configs else None
    try:
        for channel in channels:
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp.bind((host, channel.port))
            selector.register(udp, selectors.EVENT_READ)
            sockets[udp] = channel
            print(f"dual_sitl channel role={channel.role} sysid={channel.sysid} udp://{host}:{channel.port} body={channel.body_name}", file=stdout, flush=True)
        while True:
            for key, _ in selector.select(timeout=1.0):
                udp = key.fileobj
                channel = sockets[udp]
                payload, sender = udp.recvfrom(4096)
                try:
                    packet = parse_pwm_packet(payload)
                    thrusts[channel.role] = mappers[channel.role].thrusts_from_packet(packet)
                except (PacketError, ValueError) as exc:
                    print(f"invalid_pwm_packet role={channel.role} error={exc}", file=stderr, flush=True)
                    continue
                decision = clocks[channel.role].consume(packet)
                # One designated stream owns the shared clock, preventing a 2x simulation rate.
                if channel.role == "chaser" and decision.substeps:
                    world.step(thrusts, decision.substeps)
                state = world.state(channel.role)
                encoded = json.dumps(ardupilot_state_from_quad_state(state), separators=(",", ":")).encode() + b"\n"
                udp.sendto(encoded, sender)
                visualization.maybe_publish(world.state("chaser"), target_state=world.state("target"))
                if cameras is not None:
                    cameras.maybe_publish()
    finally:
        for udp in sockets:
            udp.close()
        if cameras is not None:
            cameras.close()
        visualization.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args(argv)
    try:
        serve(args.host)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
