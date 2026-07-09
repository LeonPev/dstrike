"""ROS 2 visualization publishers for the MuJoCo ground-truth state."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Iterable


WORLD_FRAME = "world"
ODOM_FRAME = "odom"
BASE_LINK_FRAME = "base_link"
GROUND_TRUTH_ODOM_TOPIC = "/sim/ground_truth/odom"
TF_TOPIC = "/tf"
SCENARIO_MARKERS_TOPIC = "/sim/scenario_markers"
DRONE_MARKERS_TOPIC = "/sim/drone_markers"
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
    """Publish ground truth odometry, TF, and simple scenario markers."""

    def __init__(
        self,
        quad: Any,
        publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ,
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
        self._site_transforms = rotor_site_transforms_from_quad(quad)
        self._drone_marker_specs = drone_marker_specs(
            site_transform.translation_xyz for site_transform in self._site_transforms
        )
        self._publish_period_s = 1.0 / publish_rate_hz
        self._last_publish_monotonic_s: float | None = None
        self._node.get_logger().info(
            "publishing simulation visualization topics "
            f"{GROUND_TRUTH_ODOM_TOPIC}, {TF_TOPIC}, {SCENARIO_MARKERS_TOPIC}, "
            f"{DRONE_MARKERS_TOPIC}"
        )

    def close(self) -> None:
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()

    def maybe_publish(self, state: Any, now_monotonic_s: float | None = None) -> bool:
        now_monotonic_s = (
            time.monotonic() if now_monotonic_s is None else now_monotonic_s
        )
        if (
            self._last_publish_monotonic_s is not None
            and now_monotonic_s - self._last_publish_monotonic_s
            < self._publish_period_s
        ):
            return False

        self.publish(state)
        self._last_publish_monotonic_s = now_monotonic_s
        return True

    def publish(self, state: Any) -> None:
        stamp = self._node.get_clock().now().to_msg()
        fields = ground_truth_odom_fields(state)

        self._odom_pub.publish(self._build_odometry(stamp, fields))
        self._tf_pub.publish(self._build_tf_message(stamp, fields))
        self._marker_pub.publish(self._build_scenario_markers(stamp))
        self._drone_marker_pub.publish(self._build_drone_markers(stamp))
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def _build_odometry(self, stamp: Any, fields: GroundTruthOdomFields) -> Any:
        odom = self._odometry_type()
        odom.header.stamp = stamp
        odom.header.frame_id = WORLD_FRAME
        odom.child_frame_id = BASE_LINK_FRAME
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

    def _build_tf_message(self, stamp: Any, fields: GroundTruthOdomFields) -> Any:
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
        markers = [
            self._landing_pad_marker(stamp),
            self._obstacle_marker(stamp, marker_id=1, x=2.0, y=-1.2),
            self._obstacle_marker(stamp, marker_id=2, x=-2.4, y=1.6),
            self._gate_marker(stamp),
            self._boundary_marker(stamp),
        ]
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

    def _landing_pad_marker(self, stamp: Any) -> Any:
        from visualization_msgs.msg import Marker

        marker = self._marker_base(stamp, "landing_pad", 0)
        marker.type = Marker.CYLINDER
        marker.pose.position.z = 0.01
        marker.scale.x = 1.2
        marker.scale.y = 1.2
        marker.scale.z = 0.02
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.35
        marker.color.a = 0.8
        return marker

    def _obstacle_marker(self, stamp: Any, marker_id: int, x: float, y: float) -> Any:
        from visualization_msgs.msg import Marker

        marker = self._marker_base(stamp, "obstacles", marker_id)
        marker.type = Marker.CUBE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.scale.x = 0.45
        marker.scale.y = 0.45
        marker.scale.z = 1.0
        marker.color.r = 0.9
        marker.color.g = 0.2
        marker.color.b = 0.1
        marker.color.a = 0.75
        return marker

    def _gate_marker(self, stamp: Any) -> Any:
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        marker = self._marker_base(stamp, "gates", 0)
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.04
        marker.color.r = 0.15
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.points = [
            Point(x=1.0, y=2.0, z=0.0),
            Point(x=1.0, y=2.0, z=1.6),
            Point(x=2.4, y=2.0, z=0.0),
            Point(x=2.4, y=2.0, z=1.6),
            Point(x=1.0, y=2.0, z=1.6),
            Point(x=2.4, y=2.0, z=1.6),
        ]
        return marker

    def _boundary_marker(self, stamp: Any) -> Any:
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        marker = self._marker_base(stamp, "scenario_boundary", 0)
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.03
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.05
        marker.color.a = 0.9
        marker.points = [
            Point(x=-5.0, y=-5.0, z=0.0),
            Point(x=5.0, y=-5.0, z=0.0),
            Point(x=5.0, y=5.0, z=0.0),
            Point(x=-5.0, y=5.0, z=0.0),
            Point(x=-5.0, y=-5.0, z=0.0),
        ]
        return marker
