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

from src.physics.ros_telemetry import (
    BASE_LINK_FRAME,
    CHASER_BASE_LINK_FRAME,
    CHASER_GROUND_TRUTH_ODOM_TOPIC,
    GROUND_TRUTH_ODOM_TOPIC,
    ODOM_FRAME,
    TARGET_BASE_LINK_FRAME,
    TARGET_ODOM_TOPIC,
    TF_TOPIC,
    WORLD_FRAME,
    FrameTransform,
    GroundTruthOdomFields,
    SimTelemetryPublisher,
    ground_truth_odom_fields,
    quaternion_wxyz_to_xyzw,
    rotor_site_transforms_from_quad,
    vector3_tuple,
)
from src.physics.scenario_config import (
    TwoDroneScenarioConfig,
    load_mountain_map_config,
)

from src.physics.scenarios import (
    ScenarioMarkerSpec,
    normalize_scenario_name,
    scenario_from_name,
)


SCENARIO_MARKERS_TOPIC = "/sim/scenario_markers"
DRONE_MARKERS_TOPIC = "/sim/drone_markers"
TERRAIN_MARKERS_TOPIC = "/sim/terrain_markers"
NATURE_MARKERS_TOPIC = "/sim/nature_markers"
DEFAULT_PUBLISH_RATE_HZ = 10.0
TRANSIENT_LOCAL_DURABILITY = "transient_local"
MARKER_ARRAY_TYPE = "visualization_msgs/msg/MarkerArray"


@dataclass(frozen=True)
class MarkerTopicContract:
    topic: str
    message_type: str
    durability: str


TWO_DRONE_MARKER_CONTRACTS = (
    MarkerTopicContract(
        SCENARIO_MARKERS_TOPIC,
        MARKER_ARRAY_TYPE,
        TRANSIENT_LOCAL_DURABILITY,
    ),
    MarkerTopicContract(
        DRONE_MARKERS_TOPIC,
        MARKER_ARRAY_TYPE,
        TRANSIENT_LOCAL_DURABILITY,
    ),
)


@dataclass(frozen=True)
class DroneMarkerSpec:
    namespace: str
    marker_id: int
    shape: str
    position_xyz: tuple[float, float, float]
    scale_xyz: tuple[float, float, float]
    color_rgba: tuple[float, float, float, float]
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    points_xyz: tuple[tuple[float, float, float], ...] = ()


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


def rotate_vector_by_quaternion(
    vector_xyz: Iterable[float],
    quaternion_xyzw: Iterable[float],
) -> tuple[float, float, float]:
    """Rotate a vector by a normalized ROS-order quaternion."""
    vector = np.asarray(vector3_tuple(vector_xyz), dtype=np.float64)
    quaternion = np.asarray(tuple(float(value) for value in quaternion_xyzw))
    if quaternion.shape != (4,):
        raise ValueError("quaternion must contain exactly four values")
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("quaternion must have finite, non-zero length")
    x, y, z, w = quaternion / norm
    quaternion_vector = np.asarray((x, y, z))
    rotated = (
        vector
        + 2.0 * w * np.cross(quaternion_vector, vector)
        + 2.0 * np.cross(quaternion_vector, np.cross(quaternion_vector, vector))
    )
    return vector3_tuple(rotated)


def world_drone_marker_specs(
    fields: GroundTruthOdomFields,
    rotor_positions: Iterable[Iterable[float]],
    *,
    role: str | None = None,
) -> tuple[DroneMarkerSpec, ...]:
    """Place the reusable quad geometry directly in the world frame."""
    if role not in {None, "target", "chaser"}:
        raise ValueError("drone marker role must be target, chaser, or None")
    role_colors = {
        "target": ((0.05, 0.62, 0.96, 1.0), (0.25, 0.86, 1.0, 0.95)),
        "chaser": ((1.0, 0.30, 0.05, 1.0), (1.0, 0.72, 0.12, 0.95)),
    }
    result: list[DroneMarkerSpec] = []
    for spec in drone_marker_specs(rotor_positions):
        offset = rotate_vector_by_quaternion(
            spec.position_xyz,
            fields.orientation_xyzw,
        )
        namespace = spec.namespace
        color = spec.color_rgba
        if role is not None:
            suffix = spec.namespace.removeprefix("drone_")
            namespace = f"{role}_{suffix}"
            body_color, rotor_color = role_colors[role]
            color = rotor_color if suffix == "rotors" else body_color
        result.append(
            DroneMarkerSpec(
                namespace=namespace,
                marker_id=spec.marker_id,
                shape=spec.shape,
                position_xyz=tuple(
                    fields.position_enu[index] + offset[index]
                    for index in range(3)
                ),
                scale_xyz=spec.scale_xyz,
                color_rgba=color,
                orientation_xyzw=fields.orientation_xyzw,
            )
        )
    return tuple(result)


def target_route_marker_points(
    config: TwoDroneScenarioConfig,
    terrain: Any,
    *,
    spacing_m: float = 2.0,
) -> tuple[tuple[float, float, float], ...]:
    """Sample the canonical first route leg at constant terrain clearance."""
    if spacing_m <= 0.0 or not math.isfinite(spacing_m):
        raise ValueError("route marker spacing must be positive and finite")
    start = config.target_drone.start_xy_m
    end = config.target_route_end_xy_m
    sample_count = max(
        2,
        int(math.ceil(config.target_drone.route_length_m / spacing_m)) + 1,
    )
    points: list[tuple[float, float, float]] = []
    for index in range(sample_count):
        fraction = index / (sample_count - 1)
        x_m = start[0] + fraction * (end[0] - start[0])
        y_m = start[1] + fraction * (end[1] - start[1])
        point = (
            x_m,
            y_m,
            float(terrain.height_at(x_m, y_m))
            + config.target_drone.altitude_agl_m,
        )
        if not all(math.isfinite(value) for value in point):
            raise ValueError("target route marker points must be finite")
        points.append(point)
    return tuple(points)


def desired_formation_position(
    target_fields: GroundTruthOdomFields,
    config: TwoDroneScenarioConfig,
) -> tuple[float, float, float]:
    """Transform the target-body formation offset into world coordinates."""
    offset = rotate_vector_by_quaternion(
        config.chaser_drone.formation_offset_target_body_m,
        target_fields.orientation_xyzw,
    )
    result = tuple(
        target_fields.position_enu[index] + offset[index]
        for index in range(3)
    )
    if not all(math.isfinite(value) for value in result):
        raise ValueError("desired formation position must be finite")
    return result


def two_drone_marker_specs(
    chaser_fields: GroundTruthOdomFields,
    target_fields: GroundTruthOdomFields,
    rotor_positions: Iterable[Iterable[float]],
    config: TwoDroneScenarioConfig,
    route_points: tuple[tuple[float, float, float], ...],
) -> tuple[DroneMarkerSpec, ...]:
    """Return distinguishable world-frame two-drone visualization geometry."""
    specs = list(
        world_drone_marker_specs(
            chaser_fields,
            rotor_positions,
            role="chaser",
        )
    )
    specs.extend(
        world_drone_marker_specs(
            target_fields,
            rotor_positions,
            role="target",
        )
    )
    specs.extend(
        (
            DroneMarkerSpec(
                namespace="target_route",
                marker_id=0,
                shape="line_strip",
                position_xyz=(0.0, 0.0, 0.0),
                scale_xyz=(0.35, 0.0, 0.0),
                color_rgba=(0.1, 0.85, 1.0, 0.9),
                points_xyz=route_points,
            ),
            DroneMarkerSpec(
                namespace="desired_formation_point",
                marker_id=0,
                shape="sphere",
                position_xyz=desired_formation_position(target_fields, config),
                scale_xyz=(1.2, 1.2, 1.2),
                color_rgba=(0.2, 1.0, 0.25, 0.95),
            ),
        )
    )
    identities = [(spec.namespace, spec.marker_id) for spec in specs]
    if len(identities) != len(set(identities)):
        raise ValueError("two-drone marker namespace/id pairs must be unique")
    return tuple(specs)


class SimVisualizationPublisher:
    """Publish ground truth odometry, TF, and selected scenario markers."""

    def __init__(
        self,
        quad: Any,
        publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ,
        scenario_markers: Iterable[ScenarioMarkerSpec] | None = None,
        publish_target: bool = False,
        two_drone_config: TwoDroneScenarioConfig | None = None,
        terrain: Any | None = None,
    ) -> None:
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if two_drone_config is not None and not publish_target:
            raise ValueError(
                "two_drone_config requires target telemetry publishing"
            )
        if two_drone_config is not None and terrain is None:
            raise ValueError("two-drone visualization requires canonical terrain")

        import rclpy
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )
        from visualization_msgs.msg import MarkerArray

        self._rclpy = rclpy
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=None)

        self._node = rclpy.create_node("sim_visualization_publisher")
        self._marker_array_type = MarkerArray
        self._telemetry = SimTelemetryPublisher(
            self._node,
            quad,
            publish_two_drone=publish_target,
        )
        marker_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
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
        self._rotor_positions = tuple(
            transform.translation_xyz for transform in self._site_transforms
        )
        drone_specs = list(drone_marker_specs(
            self._rotor_positions
        ))
        scenario_name = normalize_scenario_name(os.environ.get("SCENARIO"))
        if scenario_name == "mountain_map":
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
        self._two_drone_config = two_drone_config
        self._route_marker_points = (
            target_route_marker_points(two_drone_config, terrain)
            if two_drone_config is not None
            else ()
        )
        self._publish_period_s = 1.0 / publish_rate_hz
        self._last_publish_monotonic_s: float | None = None
        self._static_markers_published = False
        self._map_assets = self._load_map_assets()
        two_drone_topics = (
            f", {CHASER_GROUND_TRUTH_ODOM_TOPIC}, {TARGET_ODOM_TOPIC}"
            if publish_target
            else ""
        )
        self._node.get_logger().info(
            "publishing simulation visualization topics "
            f"{GROUND_TRUTH_ODOM_TOPIC}"
            f"{two_drone_topics}, "
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
        fields, target_fields = self._telemetry.publish(
            stamp,
            state,
            target_state,
        )
        self._drone_marker_pub.publish(
            self._build_drone_markers(stamp, fields, target_fields)
        )
        # Preserve the base-stack contract for volatile subscribers while the
        # transient-local writer also serves late durable subscribers.
        self._marker_pub.publish(self._build_scenario_markers(stamp))
        if not self._static_markers_published:
            if self._map_assets is not None:
                mesh, nature = self._map_assets
                self._terrain_marker_pub.publish(
                    self._build_terrain_markers(stamp, mesh)
                )
                self._nature_marker_pub.publish(
                    self._build_nature_markers(stamp, nature, mesh)
                )
            self._static_markers_published = True
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def _load_map_assets(self) -> tuple[TerrainMesh, dict[str, list[dict[str, Any]]]] | None:
        if normalize_scenario_name(os.environ.get("SCENARIO")) not in {
            "mountain_map",
            "mountain_two_drones",
        }:
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
        return self._telemetry.build_odometry(stamp, fields, child_frame_id)

    def _build_tf_message(
        self,
        stamp: Any,
        fields: GroundTruthOdomFields,
        target_fields: GroundTruthOdomFields | None = None,
    ) -> Any:
        return self._telemetry.build_tf_message(stamp, fields, target_fields)

    def _build_transform(
        self,
        stamp: Any,
        parent_frame_id: str,
        child_frame_id: str,
        translation_xyz: tuple[float, float, float],
        rotation_xyzw: tuple[float, float, float, float],
    ) -> Any:
        return self._telemetry._build_transform(
            stamp,
            parent_frame_id,
            child_frame_id,
            translation_xyz,
            rotation_xyzw,
        )

    def _build_scenario_markers(self, stamp: Any) -> Any:
        markers = [self._delete_all_scenario_markers(stamp)]
        markers.extend(
            self._scenario_shape_marker(stamp, spec)
            for spec in self._scenario_marker_specs
        )
        marker_array = self._marker_array_type()
        marker_array.markers = markers
        return marker_array

    def _build_drone_markers(
        self,
        stamp: Any,
        fields: GroundTruthOdomFields | None = None,
        target_fields: GroundTruthOdomFields | None = None,
    ) -> Any:
        if self._two_drone_config is None:
            specs = self._drone_marker_specs
            frame_id = BASE_LINK_FRAME
        else:
            if fields is None:
                raise ValueError("two-drone markers require chaser odometry fields")
            if target_fields is None:
                raise ValueError("two-drone markers require target odometry fields")
            specs = two_drone_marker_specs(
                fields,
                target_fields,
                self._rotor_positions,
                self._two_drone_config,
                self._route_marker_points,
            )
            frame_id = WORLD_FRAME
        markers = [
            self._drone_shape_marker(stamp, spec, frame_id=frame_id)
            for spec in specs
        ]
        marker_array = self._marker_array_type()
        marker_array.markers = markers
        return marker_array

    def _drone_shape_marker(
        self,
        stamp: Any,
        spec: DroneMarkerSpec,
        *,
        frame_id: str = BASE_LINK_FRAME,
    ) -> Any:
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = spec.namespace
        marker.id = spec.marker_id
        marker.action = Marker.ADD
        if spec.shape == "cube":
            marker.type = Marker.CUBE
        elif spec.shape == "cylinder":
            marker.type = Marker.CYLINDER
        elif spec.shape == "sphere":
            marker.type = Marker.SPHERE
        elif spec.shape == "line_strip":
            marker.type = Marker.LINE_STRIP
        else:
            raise ValueError(f"unsupported drone marker shape: {spec.shape}")
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
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in spec.points_xyz]
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
