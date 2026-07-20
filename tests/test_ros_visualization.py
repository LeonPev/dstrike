from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from src.physics.ros_visualization import (
    BASE_LINK_FRAME,
    TARGET_BASE_LINK_FRAME,
    FrameTransform,
    drone_marker_specs,
    ground_truth_odom_fields,
    quaternion_wxyz_to_xyzw,
    terrain_mesh,
    terrain_mesh_height_at,
    vector3_tuple,
)


@dataclass(frozen=True)
class FakeQuadState:
    position: np.ndarray
    quaternion: np.ndarray
    velocity: np.ndarray


def test_ground_truth_odom_fields_stay_in_ros_enu_z_up() -> None:
    state = FakeQuadState(
        position=np.array((1.0, 2.0, 3.0)),
        quaternion=np.array((1.0, 0.0, 0.0, 0.0)),
        velocity=np.array((4.0, 5.0, 6.0)),
    )

    fields = ground_truth_odom_fields(state)

    assert fields.position_enu == (1.0, 2.0, 3.0)
    assert fields.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert fields.linear_velocity_enu == (4.0, 5.0, 6.0)


def test_target_motion_state_uses_target_frame_and_ros_field_conventions() -> None:
    state = FakeQuadState(
        position=np.array((-90.0, 0.0, 42.0)),
        quaternion=np.array((math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))),
        velocity=np.array((0.0, 1.8, 0.0)),
    )

    fields = ground_truth_odom_fields(state)

    assert TARGET_BASE_LINK_FRAME == "target/base_link"
    assert fields.position_enu == (-90.0, 0.0, 42.0)
    assert fields.orientation_xyzw == (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    assert fields.linear_velocity_enu == (0.0, 1.8, 0.0)


def test_quaternion_wxyz_to_xyzw_normalises_and_canonicalises_sign() -> None:
    assert quaternion_wxyz_to_xyzw((-2.0, 0.0, 0.0, 0.0)) == (
        -0.0,
        -0.0,
        -0.0,
        1.0,
    )

    yaw_90_xyzw = quaternion_wxyz_to_xyzw(
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )
    assert yaw_90_xyzw == (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))


def test_vector3_tuple_rejects_wrong_dimensions() -> None:
    try:
        vector3_tuple((1.0, 2.0))
    except ValueError as exc:
        assert "exactly three" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_frame_transform_records_base_link_child_frame() -> None:
    transform = FrameTransform(
        parent_frame_id=BASE_LINK_FRAME,
        child_frame_id="rotor_front_left",
        translation_xyz=(0.18, 0.18, 0.02),
        rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert transform.parent_frame_id == "base_link"
    assert transform.child_frame_id == "rotor_front_left"


def test_drone_marker_specs_include_body_arms_and_rotors() -> None:
    specs = drone_marker_specs(
        (
            (0.18, 0.18, 0.02),
            (0.18, -0.18, 0.02),
            (-0.18, 0.18, 0.02),
            (-0.18, -0.18, 0.02),
        )
    )

    assert len(specs) == 7
    assert specs[0].namespace == "drone_body"
    assert specs[0].shape == "cube"
    assert specs[0].scale_xyz == (0.22, 0.16, 0.08)
    assert specs[3].namespace == "drone_rotors"
    assert specs[3].shape == "cylinder"
    assert specs[3].position_xyz == (0.18, 0.18, 0.02)


def test_terrain_mesh_uses_exact_decimated_cell_triangles_and_bounds() -> None:
    heights = np.arange(25, dtype=np.float32).reshape(5, 5)
    mesh = terrain_mesh(heights, size_m=40.0, decimation=2)

    assert mesh.triangle_count == 8
    assert len(mesh.points_xyz) == 24
    assert {point[0] for point in mesh.points_xyz} == {-20.0, 0.0, 20.0}
    assert {point[1] for point in mesh.points_xyz} == {-20.0, 0.0, 20.0}

    for index in range(0, len(mesh.points_xyz), 3):
        a, b, c = (np.asarray(point) for point in mesh.points_xyz[index:index + 3])
        assert np.cross(b - a, c - a)[2] > 0.0


def test_terrain_mesh_height_matches_the_displayed_coarse_triangles() -> None:
    heights = np.array(
        [
            [0.0, 100.0, 10.0, 100.0, 20.0],
            [100.0, 100.0, 100.0, 100.0, 100.0],
            [30.0, 100.0, 40.0, 100.0, 50.0],
            [100.0, 100.0, 100.0, 100.0, 100.0],
            [60.0, 100.0, 70.0, 100.0, 80.0],
        ],
        dtype=np.float32,
    )
    mesh = terrain_mesh(heights, size_m=40.0, decimation=2)

    assert terrain_mesh_height_at(mesh, -20.0, 20.0) == 0.0
    assert terrain_mesh_height_at(mesh, 20.0, -20.0) == 80.0
    assert terrain_mesh_height_at(mesh, -10.0, 10.0) == 20.0
