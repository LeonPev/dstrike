from __future__ import annotations

import math
from pathlib import Path

import mujoco

from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.mujoco_json import scenario_spawn_pose
from src.physics.scenarios import build_scenario_model_xml, scenario_from_name
from src.physics.terrain_map import TerrainMap


ROOT = Path(__file__).resolve().parents[1]


def compiled_world() -> tuple[object, MuJoCoQuad, TerrainMap]:
    scenario = scenario_from_name("mountain_two_drones")
    assert scenario.config is not None
    quad = MuJoCoQuad(
        model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario),
        vfs_files=scenario.vfs_files,
    )
    terrain = TerrainMap.from_config(
        scenario.config,
        ROOT / "models/scenarios/mountain_map",
    )
    return scenario, quad, terrain


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    return int(mujoco.mj_name2id(model, kind, name))


def test_two_drone_world_has_terrain_aware_target_and_chaser_spawns() -> None:
    scenario, quad, terrain = compiled_world()
    assert scenario.two_drone_config is not None
    config = scenario.two_drone_config
    for name, xy_m, agl_m in (
        ("target", config.target_drone.start_xy_m, config.target_drone.altitude_agl_m),
    ):
        body_id = object_id(quad.model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert body_id >= 0
        position = quad.model.body_pos[body_id]
        assert tuple(position[:2]) == xy_m
        assert math.isclose(position[2], terrain.height_at(*xy_m) + agl_m, abs_tol=1e-6)
    chaser = config.chaser_drone
    chaser_id = object_id(quad.model, mujoco.mjtObj.mjOBJ_BODY, "chaser")
    pad_id = object_id(quad.model, mujoco.mjtObj.mjOBJ_GEOM, "scenario_landing_pad")
    assert tuple(quad.model.geom_pos[pad_id][:2]) == chaser.start_xy_m
    pad_top_z = quad.model.geom_pos[pad_id][2] + quad.model.geom_size[pad_id][1]
    assert math.isclose(quad.model.body_pos[chaser_id][2], pad_top_z + chaser.initial_agl_m, abs_tol=1e-6)
    pad_bottom_z = quad.model.geom_pos[pad_id][2] - quad.model.geom_size[pad_id][1]
    pad_radius_m = quad.model.geom_size[pad_id][0]
    for dx, dy in ((0.0, 0.0), (pad_radius_m, 0.0), (-pad_radius_m, 0.0), (0.0, pad_radius_m), (0.0, -pad_radius_m)):
        assert terrain.height_at(chaser.start_xy_m[0] + dx, chaser.start_xy_m[1] + dy) <= pad_bottom_z + 1e-6


def test_target_is_kinematic_and_explicitly_non_contacting() -> None:
    _, quad, _ = compiled_world()
    target_id = object_id(quad.model, mujoco.mjtObj.mjOBJ_BODY, "target")
    assert target_id >= 0
    assert int(quad.model.body_mocapid[target_id]) >= 0
    target_geom_ids = [
        index
        for index in range(quad.model.ngeom)
        if int(quad.model.geom_bodyid[index]) == target_id
    ]
    assert target_geom_ids
    assert all(int(quad.model.geom_contype[index]) == 0 for index in target_geom_ids)
    assert all(int(quad.model.geom_conaffinity[index]) == 0 for index in target_geom_ids)


def test_public_drone_names_are_unique_and_prefixed() -> None:
    _, quad, _ = compiled_world()
    names: list[str] = []
    for kind, count in (
        (mujoco.mjtObj.mjOBJ_BODY, quad.model.nbody),
        (mujoco.mjtObj.mjOBJ_GEOM, quad.model.ngeom),
        (mujoco.mjtObj.mjOBJ_SITE, quad.model.nsite),
        (mujoco.mjtObj.mjOBJ_CAMERA, quad.model.ncam),
        (mujoco.mjtObj.mjOBJ_SENSOR, quad.model.nsensor),
        (mujoco.mjtObj.mjOBJ_ACTUATOR, quad.model.nu),
    ):
        category_names = [mujoco.mj_id2name(quad.model, kind, index) for index in range(count)]
        named = [name for name in category_names if name is not None]
        assert len(named) == len(set(named))
        names.extend(name for name in named if name.startswith(("target", "chaser")))
    assert len(names) == len(set(names))
    assert object_id(quad.model, mujoco.mjtObj.mjOBJ_CAMERA, "target_front_cam") >= 0
    assert object_id(quad.model, mujoco.mjtObj.mjOBJ_CAMERA, "chaser_front_cam") >= 0


def test_chaser_retains_free_joint_rotors_and_terrain_contact() -> None:
    _, quad, terrain = compiled_world()
    chaser_id = object_id(quad.model, mujoco.mjtObj.mjOBJ_BODY, "chaser")
    terrain_id = object_id(quad.model, mujoco.mjtObj.mjOBJ_GEOM, "mountain_terrain")
    assert chaser_id == quad.body_id
    assert object_id(quad.model, mujoco.mjtObj.mjOBJ_JOINT, "chaser_free_joint") >= 0
    assert len(quad.rotor_site_ids) == 4

    x_m, y_m = (0.0, 0.0)
    quad.reset((x_m, y_m, terrain.height_at(x_m, y_m) + 0.5))
    touched = False
    for _ in range(2000):
        mujoco.mj_step(quad.model, quad.data)
        for index in range(quad.data.ncon):
            pair = {int(quad.data.contact[index].geom1), int(quad.data.contact[index].geom2)}
            if terrain_id in pair and any(
                int(quad.model.geom_bodyid[geom_id]) == chaser_id for geom_id in pair
            ):
                touched = True
                break
        if touched:
            break
    assert touched


def test_chaser_settles_level_and_stationary_on_launch_pad() -> None:
    scenario, quad, _ = compiled_world()
    pose = scenario_spawn_pose(scenario)
    quad.reset(pose.position_xyz, pose.quaternion_wxyz)
    for _ in range(2500):
        quad.step((0.0, 0.0, 0.0, 0.0))
    body_rotation = quad.data.xmat[quad.body_id].reshape(3, 3)
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(body_rotation[2, 2])))))
    horizontal_speed_mps = math.hypot(float(quad.data.qvel[0]), float(quad.data.qvel[1]))
    assert tilt_deg < 5.0
    assert horizontal_speed_mps < 0.05


def test_ardupilot_reset_uses_configured_chaser_spawn() -> None:
    scenario = scenario_from_name("mountain_two_drones")
    assert scenario.two_drone_config is not None
    pose = scenario_spawn_pose(scenario)
    chaser = scenario.two_drone_config.chaser_drone
    assert pose.position_xyz[:2] == chaser.start_xy_m
    assert math.isclose(
        pose.position_xyz[2] - pose.launch_surface_height_m,
        chaser.initial_agl_m,
        abs_tol=1e-9,
    )


def test_default_and_mountain_map_scenarios_still_compile() -> None:
    for name in ("landing_pad", "mountain_map"):
        scenario = scenario_from_name(name)
        quad = MuJoCoQuad(
            model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario),
            vfs_files=scenario.vfs_files,
        )
        assert object_id(quad.model, mujoco.mjtObj.mjOBJ_BODY, "base_link") >= 0
        assert object_id(quad.model, mujoco.mjtObj.mjOBJ_BODY, "target") == -1
