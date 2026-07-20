"""ROS 2 visualization publishers for the MuJoCo ground-truth state."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Iterable

import numpy as np

from src.physics.scenario_config import load_mountain_map_config

from src.physics.scenarios import (
    ScenarioMarkerSpec,
    scenario_from_name,
)


WORLD_FRAME = "world"
ODOM_FRAME = "odom"
BASE_LINK_FRAME = "base_link"
TARGET_BASE_LINK_FRAME = "target/base_link"
GROUND_TRUTH_ODOM_TOPIC = "/sim/ground_truth/odom"
TARGET_ODOM_TOPIC = "/sim/target/odom"
TF_TOPIC = "/tf"
SCENARIO_MARKERS_TOPIC = "/sim/scenario_markers"
DRONE_MARKERS_TOPIC = "/sim/drone_markers"
TERRAIN_MARKERS_TOPIC = "/sim/terrain_markers"
NATURE_MARKERS_TOPIC = "/sim/nature_markers"
DEFAULT_PUBLISH_RATE_HZ = 10.0
ROTOR_SITE_NAMES = (
    "rotor_front_left",
    "rotor_front_right",
    "rotor_rear_left",
    "rotor_rear_right",
)


@dataclass(frozen=True)
class GroundTruthOdomFields:
    position_enu: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_enu: tuple[float, float, float]


@dataclass(frozen=True)
class FrameTransform:
    parent_frame_id: str
    child_frame_id: str
    translation_xyz: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class DroneMarkerSpec:
    namespace: str
    marker_id: int
    shape: str
    position_xyz: tuple[float, float, float]
    scale_xyz: tuple[float, float, float]
    color_rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class TerrainMesh:
    points_xyz: tuple[tuple[float, float, float], ...]
    triangle_count: int
    heights_m: np.ndarray
    size_m: float
    decimation: int


def terrain_mesh(heights_m: np.ndarray, size_m: float, decimation: int) -> TerrainMesh:
    """Build the canonical row-major terrain mesh used by RViz and its probe."""
    heights = np.asarray(heights_m, dtype=np.float64)
    if heights.ndim != 2 or heights.shape[0] != heights.shape[1]:
        raise ValueError("terrain height array must be square")
    if decimation <= 0 or (heights.shape[0] - 1) % decimation:
        raise ValueError("decimation must divide terrain resolution - 1")
    radius = float(size_m) / 2.0
    coordinates = np.linspace(-radius, radius, heights.shape[0])
    points: list[tuple[float, float, float]] = []
    for row in range(0, heights.shape[0] - 1, decimation):
        for col in range(0, heights.shape[1] - 1, decimation):
            r1, c1 = row + decimation, col + decimation
            p00 = (coordinates[col], coordinates[-1 - row], float(heights[row, col]))
            p10 = (coordinates[c1], coordinates[-1 - row], float(heights[row, c1]))
            p01 = (coordinates[col], coordinates[-1 - r1], float(heights[r1, col]))
            p11 = (coordinates[c1], coordinates[-1 - r1], float(heights[r1, c1]))
            # Counter-clockwise from above so Ogre/RViz lights the visible face.
            points.extend((p00, p01, p10, p11, p10, p01))
    return TerrainMesh(
        tuple(points), len(points) // 3, heights, float(size_m), decimation
    )


def terrain_mesh_height_at(mesh: TerrainMesh, x_m: float, y_m: float) -> float:
    """Return the height of the displayed decimated triangle surface."""
    radius = mesh.size_m / 2.0
    if not (-radius <= x_m <= radius and -radius <= y_m <= radius):
        raise ValueError("terrain mesh query is outside map bounds")
    resolution = mesh.heights_m.shape[0]
    col = (x_m + radius) * (resolution - 1) / mesh.size_m
    row = (radius - y_m) * (resolution - 1) / mesh.size_m
    c0 = min(int(math.floor(col / mesh.decimation)) * mesh.decimation, resolution - 1 - mesh.decimation)
    r0 = min(int(math.floor(row / mesh.decimation)) * mesh.decimation, resolution - 1 - mesh.decimation)
    tx = (col - c0) / mesh.decimation
    ty = (row - r0) / mesh.decimation
    c1, r1 = c0 + mesh.decimation, r0 + mesh.decimation
    h00 = float(mesh.heights_m[r0, c0])
    h10 = float(mesh.heights_m[r0, c1])
    h01 = float(mesh.heights_m[r1, c0])
    h11 = float(mesh.heights_m[r1, c1])
    if tx + ty <= 1.0:
        return h00 + tx * (h10 - h00) + ty * (h01 - h00)
    return h11 + (1.0 - tx) * (h01 - h11) + (1.0 - ty) * (h10 - h11)


def ground_truth_odom_fields(state: Any) -> GroundTruthOdomFields:
    """Extract ROS ENU/z-up odometry fields from a MuJoCo quad state."""
    return GroundTruthOdomFields(
        position_enu=vector3_tuple(state.position),
        orientation_xyzw=quaternion_wxyz_to_xyzw(state.quaternion),
        linear_velocity_enu=vector3_tuple(state.velocity),
    )


def rotor_site_transforms_from_quad(quad: Any) -> tuple[FrameTransform, ...]:
    """Return base_link-relative transforms for the named rotor sites."""
    transforms: list[FrameTransform] = []
    for site_name, site_id in zip(ROTOR_SITE_NAMES, quad.rotor_site_ids):
        site_position = vector3_tuple(quad.model.site_pos[site_id])
        transforms.append(
            FrameTransform(
                parent_frame_id=BASE_LINK_FRAME,
                child_frame_id=site_name,
                translation_xyz=site_position,
                rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
        )
    return tuple(transforms)


def drone_marker_specs(
    rotor_positions: Iterable[Iterable[float]],
) -> tuple[DroneMarkerSpec, ...]:
    """Return base_link-relative geometry for a simple RViz quad body."""
    specs = [
        DroneMarkerSpec(
            namespace="drone_body",
            marker_id=0,
            shape="cube",
            position_xyz=(0.0, 0.0, 0.0),
            scale_xyz=(0.22, 0.16, 0.08),
            color_rgba=(0.08, 0.10, 0.12, 1.0),
        ),
        DroneMarkerSpec(
            namespace="drone_body",
            marker_id=1,
            shape="cube",
            position_xyz=(0.0, 0.0, 0.0),
            scale_xyz=(0.55, 0.035, 0.035),
            color_rgba=(0.65, 0.72, 0.78, 1.0),
        ),
        DroneMarkerSpec(
            namespace="drone_body",
            marker_id=2,
            shape="cube",
            position_xyz=(0.0, 0.0, 0.0),
            scale_xyz=(0.035, 0.55, 0.035),
            color_rgba=(0.65, 0.72, 0.78, 1.0),
        ),
    ]
    rotor_colors = (
        (0.05, 0.35, 0.95, 0.95),
        (0.95, 0.15, 0.10, 0.95),
        (0.95, 0.15, 0.10, 0.95),
        (0.05, 0.35, 0.95, 0.95),
    )
    for index, position in enumerate(rotor_positions):
        specs.append(
            DroneMarkerSpec(
                namespace="drone_rotors",
                marker_id=index,
                shape="cylinder",
                position_xyz=vector3_tuple(position),
                scale_xyz=(0.14, 0.14, 0.02),
                color_rgba=rotor_colors[index % len(rotor_colors)],
            )
        )
    return tuple(specs)


def vector3_tuple(values: Iterable[float]) -> tuple[float, float, float]:
    value_tuple = tuple(float(value) for value in values)
    if len(value_tuple) != 3:
        raise ValueError("vector must contain exactly three values")
    return value_tuple


def quaternion_wxyz_to_xyzw(
    values: Iterable[float],
) -> tuple[float, float, float, float]:
    value_tuple = tuple(float(value) for value in values)
    if len(value_tuple) != 4:
        raise ValueError("quaternion must contain exactly four values")
    w, x, y, z = value_tuple
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        raise ValueError("quaternion must have non-zero length")
    w /= norm
    x /= norm
    y /= norm
    z /= norm
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    return (x, y, z, w)


class SimVisualizationPublisher:
    """Publish ground truth odometry, TF, and selected scenario markers."""

    def __init__(
        self,
        quad: Any,
        publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ,
        scenario_markers: Iterable[ScenarioMarkerSpec] | None = None,
        publish_target: bool = False,
    ) -> None:
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile
        from tf2_msgs.msg import TFMessage
        from visualization_msgs.msg import MarkerArray

        self._rclpy = rclpy
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=None)

        self._node = rclpy.create_node("sim_visualization_publisher")
        self._odometry_type = Odometry
        self._tf_message_type = TFMessage
        self._marker_array_type = MarkerArray
        self._odom_pub = self._node.create_publisher(
            Odometry,
            GROUND_TRUTH_ODOM_TOPIC,
            10,
        )
        self._target_odom_pub = (
            self._node.create_publisher(Odometry, TARGET_ODOM_TOPIC, 10)
            if publish_target
            else None
        )
        self._tf_pub = self._node.create_publisher(TFMessage, TF_TOPIC, 10)
        marker_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self._node.create_publisher(
            MarkerArray,
            SCENARIO_MARKERS_TOPIC,
            marker_qos,
        )
        self._drone_marker_pub = self._node.create_publisher(
            MarkerArray,
            DRONE_MARKERS_TOPIC,
            marker_qos,
        )
        self._terrain_marker_pub = self._node.create_publisher(
            MarkerArray, TERRAIN_MARKERS_TOPIC, marker_qos
        )
        self._nature_marker_pub = self._node.create_publisher(
            MarkerArray, NATURE_MARKERS_TOPIC, marker_qos
        )
        self._site_transforms = rotor_site_transforms_from_quad(quad)
        drone_specs = list(drone_marker_specs(
            site_transform.translation_xyz for site_transform in self._site_transforms
        ))
        if os.environ.get("SCENARIO", "landing_pad") == "mountain_map":
            drone_specs.append(
                DroneMarkerSpec(
                    namespace="drone_locator",
                    marker_id=0,
                    shape="cylinder",
                    position_xyz=(0.0, 0.0, -0.15),
                    scale_xyz=(5.0, 5.0, 0.15),
                    color_rgba=(1.0, 0.78, 0.0, 0.85),
                )
            )
        self._drone_marker_specs = tuple(drone_specs)
        self._scenario_marker_specs = tuple(
            scenario_from_name(None).marker_specs
            if scenario_markers is None
            else scenario_markers
        )
        self._publish_period_s = 1.0 / publish_rate_hz
        self._last_publish_monotonic_s: float | None = None
        self._static_markers_published = False
        self._map_assets = self._load_map_assets()
        self._node.get_logger().info(
            "publishing simulation visualization topics "
            f"{GROUND_TRUTH_ODOM_TOPIC}"
            f"{', ' + TARGET_ODOM_TOPIC if publish_target else ''}, "
            f"{TF_TOPIC}, {SCENARIO_MARKERS_TOPIC}, "
            f"{DRONE_MARKERS_TOPIC}, {TERRAIN_MARKERS_TOPIC}, {NATURE_MARKERS_TOPIC}"
        )

    def close(self) -> None:
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()

    def maybe_publish(
        self,
        state: Any,
        now_monotonic_s: float | None = None,
        target_state: Any | None = None,
    ) -> bool:
        now_monotonic_s = (
            time.monotonic() if now_monotonic_s is None else now_monotonic_s
        )
        if (
            self._last_publish_monotonic_s is not None
            and now_monotonic_s - self._last_publish_monotonic_s
            < self._publish_period_s
        ):
            return False

        self.publish(state, target_state=target_state)
        self._last_publish_monotonic_s = now_monotonic_s
        return True

    def publish(self, state: Any, target_state: Any | None = None) -> None:
        stamp = self._node.get_clock().now().to_msg()
        fields = ground_truth_odom_fields(state)
        target_fields = (
            ground_truth_odom_fields(target_state)
            if target_state is not None
            else None
        )

        self._odom_pub.publish(self._build_odometry(stamp, fields))
        if self._target_odom_pub is not None:
            if target_fields is None:
                raise ValueError("target state is required when target publishing is enabled")
            self._target_odom_pub.publish(
                self._build_odometry(
                    stamp,
                    target_fields,
                    child_frame_id=TARGET_BASE_LINK_FRAME,
                )
            )
        self._tf_pub.publish(self._build_tf_message(stamp, fields, target_fields))
        self._marker_pub.publish(self._build_scenario_markers(stamp))
        self._drone_marker_pub.publish(self._build_drone_markers(stamp))
        if not self._static_markers_published and self._map_assets is not None:
            mesh, nature = self._map_assets
            self._terrain_marker_pub.publish(self._build_terrain_markers(stamp, mesh))
            self._nature_marker_pub.publish(
                self._build_nature_markers(stamp, nature, mesh)
            )
            self._static_markers_published = True
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def _load_map_assets(self) -> tuple[TerrainMesh, dict[str, list[dict[str, Any]]]] | None:
        if os.environ.get("SCENARIO", "landing_pad") != "mountain_map":
            return None
        asset_dir = Path("/workspace/models/scenarios/mountain_map")
        if not asset_dir.exists():
            asset_dir = Path("models/scenarios/mountain_map")
        config = load_mountain_map_config(asset_dir / "scenario.yaml")
        mesh = terrain_mesh(
            np.load(asset_dir / "heightfield_m.npy"),
            config.terrain.size_m,
            config.terrain.rviz_decimation,
        )
        with (asset_dir / "nature.json").open(encoding="utf-8") as nature_file:
            nature = json.load(nature_file)
        return mesh, nature

    def _build_terrain_markers(self, stamp: Any, mesh: TerrainMesh) -> Any:
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        marker = self._marker_base(stamp, "terrain", 0)
        marker.type = Marker.TRIANGLE_LIST
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.30, 0.55, 0.22, 1.0)
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in mesh.points_xyz]
        result = self._marker_array_type()
        result.markers = [marker]
        return result

    def _build_nature_markers(
        self,
        stamp: Any,
        nature: dict[str, list[dict[str, Any]]],
        mesh: TerrainMesh,
    ) -> Any:
        from visualization_msgs.msg import Marker

        markers = []
        for namespace, marker_type, color in (
            ("trees", Marker.CYLINDER, (0.12, 0.42, 0.10, 1.0)),
            ("rocks", Marker.SPHERE, (0.38, 0.35, 0.32, 1.0)),
        ):
            for marker_id, item in enumerate(nature[namespace]):
                marker = self._marker_base(stamp, namespace, marker_id)
                marker.type = marker_type
                scale = float(item["scale"])
                height = (5.0 if namespace == "trees" else 1.5) * scale
                width = (1.2 if namespace == "trees" else 2.0) * scale
                marker.pose.position.x = float(item["x_m"])
                marker.pose.position.y = float(item["y_m"])
                # nature.json remains aligned to the full physics heightfield.
                # Anchor the visual marker to the decimated mesh shown by RViz,
                # otherwise the simplification can leave a visible air gap.
                visual_ground_z = terrain_mesh_height_at(
                    mesh, marker.pose.position.x, marker.pose.position.y
                )
                marker.pose.position.z = visual_ground_z + height / 2.0
                marker.scale.x = marker.scale.y = width
                marker.scale.z = height
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
                markers.append(marker)
        result = self._marker_array_type()
        result.markers = markers
        return result

    def _build_odometry(
        self,
        stamp: Any,
        fields: GroundTruthOdomFields,
        child_frame_id: str = BASE_LINK_FRAME,
    ) -> Any:
        odom = self._odometry_type()
        odom.header.stamp = stamp
        odom.header.frame_id = WORLD_FRAME
        odom.child_frame_id = child_frame_id
        odom.pose.pose.position.x = fields.position_enu[0]
        odom.pose.pose.position.y = fields.position_enu[1]
        odom.pose.pose.position.z = fields.position_enu[2]
        odom.pose.pose.orientation.x = fields.orientation_xyzw[0]
        odom.pose.pose.orientation.y = fields.orientation_xyzw[1]
        odom.pose.pose.orientation.z = fields.orientation_xyzw[2]
        odom.pose.pose.orientation.w = fields.orientation_xyzw[3]
        odom.twist.twist.linear.x = fields.linear_velocity_enu[0]
        odom.twist.twist.linear.y = fields.linear_velocity_enu[1]
        odom.twist.twist.linear.z = fields.linear_velocity_enu[2]
        return odom

    def _build_tf_message(
        self,
        stamp: Any,
        fields: GroundTruthOdomFields,
        target_fields: GroundTruthOdomFields | None = None,
    ) -> Any:
        transforms = [
            self._build_transform(
                stamp=stamp,
                parent_frame_id=WORLD_FRAME,
                child_frame_id=ODOM_FRAME,
                translation_xyz=(0.0, 0.0, 0.0),
                rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            ),
            self._build_transform(
                stamp=stamp,
                parent_frame_id=WORLD_FRAME,
                child_frame_id=BASE_LINK_FRAME,
                translation_xyz=fields.position_enu,
                rotation_xyzw=fields.orientation_xyzw,
            )
        ]
        if target_fields is not None:
            transforms.append(
                self._build_transform(
                    stamp=stamp,
                    parent_frame_id=WORLD_FRAME,
                    child_frame_id=TARGET_BASE_LINK_FRAME,
                    translation_xyz=target_fields.position_enu,
                    rotation_xyzw=target_fields.orientation_xyzw,
                )
            )
        for site_transform in self._site_transforms:
            transforms.append(
                self._build_transform(
                    stamp=stamp,
                    parent_frame_id=site_transform.parent_frame_id,
                    child_frame_id=site_transform.child_frame_id,
                    translation_xyz=site_transform.translation_xyz,
                    rotation_xyzw=site_transform.rotation_xyzw,
                )
            )
        message = self._tf_message_type()
        message.transforms = transforms
        return message

    def _build_transform(
        self,
        stamp: Any,
        parent_frame_id: str,
        child_frame_id: str,
        translation_xyz: tuple[float, float, float],
        rotation_xyzw: tuple[float, float, float, float],
    ) -> Any:
        from geometry_msgs.msg import TransformStamped

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame_id
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = translation_xyz[0]
        transform.transform.translation.y = translation_xyz[1]
        transform.transform.translation.z = translation_xyz[2]
        transform.transform.rotation.x = rotation_xyzw[0]
        transform.transform.rotation.y = rotation_xyzw[1]
        transform.transform.rotation.z = rotation_xyzw[2]
        transform.transform.rotation.w = rotation_xyzw[3]
        return transform

    def _build_scenario_markers(self, stamp: Any) -> Any:
        markers = [self._delete_all_scenario_markers(stamp)]
        markers.extend(
            self._scenario_shape_marker(stamp, spec)
            for spec in self._scenario_marker_specs
        )
        marker_array = self._marker_array_type()
        marker_array.markers = markers
        return marker_array

    def _build_drone_markers(self, stamp: Any) -> Any:
        markers = [
            self._drone_shape_marker(stamp, spec) for spec in self._drone_marker_specs
        ]
        marker_array = self._marker_array_type()
        marker_array.markers = markers
        return marker_array

    def _drone_shape_marker(self, stamp: Any, spec: DroneMarkerSpec) -> Any:
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = BASE_LINK_FRAME
        marker.ns = spec.namespace
        marker.id = spec.marker_id
        marker.action = Marker.ADD
        if spec.shape == "cube":
            marker.type = Marker.CUBE
        elif spec.shape == "cylinder":
            marker.type = Marker.CYLINDER
        else:
            raise ValueError(f"unsupported drone marker shape: {spec.shape}")
        marker.pose.position.x = spec.position_xyz[0]
        marker.pose.position.y = spec.position_xyz[1]
        marker.pose.position.z = spec.position_xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = spec.scale_xyz[0]
        marker.scale.y = spec.scale_xyz[1]
        marker.scale.z = spec.scale_xyz[2]
        marker.color.r = spec.color_rgba[0]
        marker.color.g = spec.color_rgba[1]
        marker.color.b = spec.color_rgba[2]
        marker.color.a = spec.color_rgba[3]
        return marker

    def _marker_base(self, stamp: Any, namespace: str, marker_id: int) -> Any:
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = WORLD_FRAME
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _delete_all_scenario_markers(self, stamp: Any) -> Any:
        from visualization_msgs.msg import Marker

        marker = self._marker_base(stamp, "scenario", 0)
        marker.action = Marker.DELETEALL
        return marker

    def _scenario_shape_marker(self, stamp: Any, spec: ScenarioMarkerSpec) -> Any:
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        marker = self._marker_base(stamp, spec.namespace, spec.marker_id)
        if spec.shape == "cube":
            marker.type = Marker.CUBE
        elif spec.shape == "cylinder":
            marker.type = Marker.CYLINDER
        elif spec.shape == "line_list":
            marker.type = Marker.LINE_LIST
        elif spec.shape == "line_strip":
            marker.type = Marker.LINE_STRIP
        else:
            raise ValueError(f"unsupported scenario marker shape: {spec.shape}")
        marker.pose.position.x = spec.position_xyz[0]
        marker.pose.position.y = spec.position_xyz[1]
        marker.pose.position.z = spec.position_xyz[2]
        marker.pose.orientation.x = spec.orientation_xyzw[0]
        marker.pose.orientation.y = spec.orientation_xyzw[1]
        marker.pose.orientation.z = spec.orientation_xyzw[2]
        marker.pose.orientation.w = spec.orientation_xyzw[3]
        marker.scale.x = spec.scale_xyz[0]
        marker.scale.y = spec.scale_xyz[1]
        marker.scale.z = spec.scale_xyz[2]
        marker.color.r = spec.color_rgba[0]
        marker.color.g = spec.color_rgba[1]
        marker.color.b = spec.color_rgba[2]
        marker.color.a = spec.color_rgba[3]
        marker.points = [
            Point(x=point[0], y=point[1], z=point[2])
            for point in spec.points_xyz
        ]
        return marker
