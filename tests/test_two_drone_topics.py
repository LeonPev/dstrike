from __future__ import annotations

import math
from pathlib import Path

from src.physics.ros_telemetry import (
    BASE_LINK_FRAME,
    CHASER_BASE_LINK_FRAME,
    CHASER_FRONT_CAMERA_FRAME,
    CHASER_GROUND_TRUTH_ODOM_TOPIC,
    DRONE_ODOM_TOPIC,
    GROUND_TRUTH_ODOM_TOPIC,
    ODOMETRY_TYPE,
    TARGET_BASE_LINK_FRAME,
    TARGET_FRONT_CAMERA_FRAME,
    TARGET_ODOM_TOPIC,
    TF_TOPIC,
    TF_TYPE,
    TWO_DRONE_TELEMETRY_CONTRACTS,
    VOLATILE_DURABILITY,
    GroundTruthOdomFields,
    odom_pose_errors,
    two_drone_camera_transforms_from_quad,
)
from src.physics.ros_visualization import (
    DRONE_MARKERS_TOPIC,
    MARKER_ARRAY_TYPE,
    SCENARIO_MARKERS_TOPIC,
    TRANSIENT_LOCAL_DURABILITY,
    TWO_DRONE_MARKER_CONTRACTS,
    WORLD_FRAME,
    desired_formation_position,
    target_route_marker_points,
    two_drone_marker_specs,
)
from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.scenarios import (
    SCENARIOS_DIR,
    build_scenario_model_xml,
    scenario_from_name,
)
from src.physics.terrain_map import TerrainMap


ROTOR_POSITIONS = (
    (0.18, 0.18, 0.02),
    (0.18, -0.18, 0.02),
    (-0.18, 0.18, 0.02),
    (-0.18, -0.18, 0.02),
)
PROJECT_ROOT = Path(__file__).resolve().parents[1]


def canonical_config_and_terrain():
    scenario = scenario_from_name("mountain_two_drones")
    assert scenario.config is not None
    assert scenario.two_drone_config is not None
    terrain = TerrainMap.from_config(
        scenario.config,
        SCENARIOS_DIR / "mountain_map",
    )
    return scenario.two_drone_config, terrain


def fields(
    position: tuple[float, float, float],
    yaw_deg: float,
) -> GroundTruthOdomFields:
    half_yaw = math.radians(yaw_deg) / 2.0
    return GroundTruthOdomFields(
        position_enu=position,
        orientation_xyzw=(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)),
        linear_velocity_enu=(0.0, 0.0, 0.0),
    )


def test_two_drone_topic_contract_keeps_aliases_live_and_volatile() -> None:
    contracts = {contract.topic: contract for contract in TWO_DRONE_TELEMETRY_CONTRACTS}

    assert set(contracts) == {
        TARGET_ODOM_TOPIC,
        CHASER_GROUND_TRUTH_ODOM_TOPIC,
        GROUND_TRUTH_ODOM_TOPIC,
        DRONE_ODOM_TOPIC,
        TF_TOPIC,
    }
    for topic in (
        TARGET_ODOM_TOPIC,
        CHASER_GROUND_TRUTH_ODOM_TOPIC,
        GROUND_TRUTH_ODOM_TOPIC,
        DRONE_ODOM_TOPIC,
    ):
        assert contracts[topic].message_type == ODOMETRY_TYPE
        assert contracts[topic].durability == VOLATILE_DURABILITY
    assert contracts[TF_TOPIC].message_type == TF_TYPE
    assert {SCENARIO_MARKERS_TOPIC, DRONE_MARKERS_TOPIC} == {
        "/sim/scenario_markers",
        "/sim/drone_markers",
    }
    marker_contracts = {
        contract.topic: contract for contract in TWO_DRONE_MARKER_CONTRACTS
    }
    assert set(marker_contracts) == {
        SCENARIO_MARKERS_TOPIC,
        DRONE_MARKERS_TOPIC,
    }
    assert all(
        contract.message_type == MARKER_ARRAY_TYPE
        and contract.durability == TRANSIENT_LOCAL_DURABILITY
        for contract in marker_contracts.values()
    )


def test_namespaced_frames_extend_instead_of_replacing_legacy_base_link() -> None:
    assert WORLD_FRAME == "world"
    assert BASE_LINK_FRAME == "base_link"
    assert CHASER_BASE_LINK_FRAME == "chaser/base_link"
    assert TARGET_BASE_LINK_FRAME == "target/base_link"
    assert CHASER_FRONT_CAMERA_FRAME == "chaser/front_camera"
    assert TARGET_FRONT_CAMERA_FRAME == "target/front_camera"


def test_compose_keeps_mavlink_odom_owner_and_selects_sitl_truth_input() -> None:
    compose = (PROJECT_ROOT / "compose.yaml").read_text(encoding="utf-8")
    bridge = (
        PROJECT_ROOT
        / "src/ros2/drone_bridge/drone_bridge/mavlink_bridge.py"
    ).read_text(encoding="utf-8")

    assert 'NORMALIZED_SCENARIO="$${SCENARIO//-/_}"' in compose
    assert 'if [ "$${NORMALIZED_SCENARIO}" = "mountain_two_drones" ]' in compose
    assert '-p use_sim_state:="$${USE_SIM_STATE}"' in compose
    assert 'self.declare_parameter("use_sim_state", False)' in bridge
    assert '-p odom_frame_id:=world' in compose
    assert '-p odom_alignment_topic:=/sim/chaser/ground_truth/odom' in compose
    assert 'self.declare_parameter("odom_alignment_topic", "")' in bridge
    assert 'self.create_publisher(Odometry, "/drone/odom", 10)' in bridge


def test_compiled_front_camera_transforms_use_namespaced_body_parents() -> None:
    scenario = scenario_from_name("mountain_two_drones")
    quad = MuJoCoQuad(
        model_xml=build_scenario_model_xml(DEFAULT_MODEL_PATH, scenario),
        vfs_files=scenario.vfs_files,
    )

    transforms = two_drone_camera_transforms_from_quad(quad)

    by_child = {transform.child_frame_id: transform for transform in transforms}
    assert by_child[CHASER_FRONT_CAMERA_FRAME].parent_frame_id == CHASER_BASE_LINK_FRAME
    assert by_child[TARGET_FRONT_CAMERA_FRAME].parent_frame_id == TARGET_BASE_LINK_FRAME
    assert all(
        math.isfinite(value)
        for transform in transforms
        for values in (transform.translation_xyz, transform.rotation_xyzw)
        for value in values
    )
    assert all(
        math.isclose(actual, expected, abs_tol=1e-12)
        for transform in transforms
        for actual, expected in zip(
            transform.translation_xyz,
            (0.24, 0.0, 0.10),
        )
    )


def test_alias_pose_errors_are_wrap_safe() -> None:
    chaser = fields((-120.0, -25.0, 2.0), 359.8)
    alias = fields((-119.98, -25.01, 2.0), 0.2)

    position_error_m, yaw_error_deg = odom_pose_errors(chaser, alias)

    assert position_error_m < 0.05
    assert math.isclose(yaw_error_deg, 0.4, abs_tol=1e-12)


def test_route_and_two_drone_markers_use_canonical_world_geometry() -> None:
    config, terrain = canonical_config_and_terrain()
    route_points = target_route_marker_points(config, terrain)
    target_z = (
        terrain.height_at(*config.target_drone.start_xy_m)
        + config.target_drone.altitude_agl_m
    )
    target = fields(
        (
            config.target_drone.start_xy_m[0],
            config.target_drone.start_xy_m[1],
            target_z,
        ),
        config.target_drone.heading_deg,
    )
    chaser = fields((-120.0, -25.0, 1.0), 0.0)

    specs = two_drone_marker_specs(
        chaser,
        target,
        ROTOR_POSITIONS,
        config,
        route_points,
    )

    assert route_points[0][:2] == config.target_drone.start_xy_m
    assert all(
        math.isclose(actual, expected, abs_tol=1e-9)
        for actual, expected in zip(
            route_points[-1][:2],
            config.target_route_end_xy_m,
        )
    )
    assert all(
        math.isclose(
            point[2] - terrain.height_at(point[0], point[1]),
            config.target_drone.altitude_agl_m,
            abs_tol=1e-9,
        )
        for point in route_points
    )
    namespaces = {spec.namespace for spec in specs}
    assert {"chaser_body", "target_body"}.issubset(namespaces)
    assert {"target_route", "desired_formation_point"}.issubset(namespaces)
    identities = [(spec.namespace, spec.marker_id) for spec in specs]
    assert len(identities) == len(set(identities))
    assert all(
        math.isfinite(value)
        for spec in specs
        for values in (
            spec.position_xyz,
            spec.orientation_xyzw,
            spec.scale_xyz,
            spec.color_rgba,
            *spec.points_xyz,
        )
        for value in values
    )
    chaser_color = next(spec.color_rgba for spec in specs if spec.namespace == "chaser_body")
    target_color = next(spec.color_rgba for spec in specs if spec.namespace == "target_body")
    assert chaser_color != target_color


def test_formation_offset_rotates_with_target_body_heading() -> None:
    config, _ = canonical_config_and_terrain()
    target = fields((10.0, 20.0, 30.0), 90.0)

    formation = desired_formation_position(target, config)

    # Body offset [-8, -5, 0] rotated +90 degrees becomes [5, -8, 0].
    assert all(
        math.isclose(actual, expected, abs_tol=1e-9)
        for actual, expected in zip(formation, (15.0, 12.0, 30.0))
    )
