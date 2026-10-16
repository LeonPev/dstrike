"""ROS 2 ground-truth telemetry for simulated vehicles.

This module is deliberately limited to translating simulator state into the
public odometry and TF contract.  Flight-control telemetry from a real vehicle
continues to enter ROS through the MAVLink bridge, which keeps the simulator
implementation out of the hardware adapter path.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable


WORLD_FRAME = "world"
ODOM_FRAME = "odom"
BASE_LINK_FRAME = "base_link"
CHASER_BASE_LINK_FRAME = "chaser/base_link"
TARGET_BASE_LINK_FRAME = "target/base_link"
CHASER_FRONT_CAMERA_FRAME = "chaser/front_camera"
TARGET_FRONT_CAMERA_FRAME = "target/front_camera"

GROUND_TRUTH_ODOM_TOPIC = "/sim/ground_truth/odom"
CHASER_GROUND_TRUTH_ODOM_TOPIC = "/sim/chaser/ground_truth/odom"
TARGET_ODOM_TOPIC = "/sim/target/odom"
DRONE_ODOM_TOPIC = "/drone/odom"
TF_TOPIC = "/tf"

ODOMETRY_TYPE = "nav_msgs/msg/Odometry"
TF_TYPE = "tf2_msgs/msg/TFMessage"
VOLATILE_DURABILITY = "volatile"


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
class TelemetryTopicContract:
    topic: str
    message_type: str
    durability: str


TWO_DRONE_TELEMETRY_CONTRACTS = (
    TelemetryTopicContract(TARGET_ODOM_TOPIC, ODOMETRY_TYPE, VOLATILE_DURABILITY),
    TelemetryTopicContract(
        CHASER_GROUND_TRUTH_ODOM_TOPIC,
        ODOMETRY_TYPE,
        VOLATILE_DURABILITY,
    ),
    TelemetryTopicContract(
        GROUND_TRUTH_ODOM_TOPIC,
        ODOMETRY_TYPE,
        VOLATILE_DURABILITY,
    ),
    # This independent chaser alias remains owned by the MAVLink bridge.
    TelemetryTopicContract(DRONE_ODOM_TOPIC, ODOMETRY_TYPE, VOLATILE_DURABILITY),
    TelemetryTopicContract(TF_TOPIC, TF_TYPE, VOLATILE_DURABILITY),
)


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
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("quaternion must have finite, non-zero length")
    w /= norm
    x /= norm
    y /= norm
    z /= norm
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    return (x, y, z, w)


def ground_truth_odom_fields(state: Any) -> GroundTruthOdomFields:
    """Extract ROS ENU/z-up odometry fields from simulator state."""
    fields = GroundTruthOdomFields(
        position_enu=vector3_tuple(state.position),
        orientation_xyzw=quaternion_wxyz_to_xyzw(state.quaternion),
        linear_velocity_enu=vector3_tuple(state.velocity),
    )
    if not all(
        math.isfinite(value)
        for values in (
            fields.position_enu,
            fields.orientation_xyzw,
            fields.linear_velocity_enu,
        )
        for value in values
    ):
        raise ValueError("ground-truth odometry fields must be finite")
    return fields


def rotor_site_transforms_from_quad(quad: Any) -> tuple[FrameTransform, ...]:
    """Return legacy base_link-relative transforms for the four rotor sites."""
    transforms: list[FrameTransform] = []
    for site_name, site_id in zip(
        (
            "rotor_front_left",
            "rotor_front_right",
            "rotor_rear_left",
            "rotor_rear_right",
        ),
        quad.rotor_site_ids,
    ):
        transforms.append(
            FrameTransform(
                parent_frame_id=BASE_LINK_FRAME,
                child_frame_id=site_name,
                translation_xyz=vector3_tuple(quad.model.site_pos[site_id]),
                rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
        )
    return tuple(transforms)


def two_drone_camera_transforms_from_quad(
    quad: Any,
) -> tuple[FrameTransform, FrameTransform]:
    """Return body-relative transforms for the compiled front cameras."""
    import mujoco

    transforms: list[FrameTransform] = []
    for camera_name, parent_frame, child_frame in (
        (
            "chaser_front_cam",
            CHASER_BASE_LINK_FRAME,
            CHASER_FRONT_CAMERA_FRAME,
        ),
        (
            "target_front_cam",
            TARGET_BASE_LINK_FRAME,
            TARGET_FRONT_CAMERA_FRAME,
        ),
    ):
        camera_id = int(
            mujoco.mj_name2id(
                quad.model,
                mujoco.mjtObj.mjOBJ_CAMERA,
                camera_name,
            )
        )
        if camera_id < 0:
            raise ValueError(f"two-drone model must define camera {camera_name}")
        transforms.append(
            FrameTransform(
                parent_frame_id=parent_frame,
                child_frame_id=child_frame,
                translation_xyz=vector3_tuple(quad.model.cam_pos[camera_id]),
                rotation_xyzw=quaternion_wxyz_to_xyzw(
                    quad.model.cam_quat[camera_id]
                ),
            )
        )
    return (transforms[0], transforms[1])


def quaternion_yaw_deg(values_xyzw: Iterable[float]) -> float:
    """Return z-up yaw for a general roll/pitch/yaw quaternion."""
    values = tuple(float(value) for value in values_xyzw)
    if len(values) != 4:
        raise ValueError("quaternion must contain exactly four values")
    x, y, z, w = values
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("quaternion must have finite, non-zero length")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(sin_yaw, cos_yaw)) % 360.0


def odom_pose_errors(
    first: GroundTruthOdomFields,
    second: GroundTruthOdomFields,
) -> tuple[float, float]:
    """Return position and wrapped-yaw differences for alias verification."""
    position_error_m = math.dist(first.position_enu, second.position_enu)
    first_yaw = quaternion_yaw_deg(first.orientation_xyzw)
    second_yaw = quaternion_yaw_deg(second.orientation_xyzw)
    yaw_error_deg = abs((first_yaw - second_yaw + 180.0) % 360.0 - 180.0)
    return position_error_m, yaw_error_deg


class SimTelemetryPublisher:
    """Publish simulator-owned odometry and transforms on an existing node."""

    def __init__(self, node: Any, quad: Any, *, publish_two_drone: bool) -> None:
        from nav_msgs.msg import Odometry
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )
        from tf2_msgs.msg import TFMessage

        self._node = node
        self._odometry_type = Odometry
        self._tf_message_type = TFMessage
        self._publish_two_drone = publish_two_drone
        live_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._ground_truth_pub = node.create_publisher(
            Odometry,
            GROUND_TRUTH_ODOM_TOPIC,
            live_qos,
        )
        self._chaser_ground_truth_pub = (
            node.create_publisher(
                Odometry,
                CHASER_GROUND_TRUTH_ODOM_TOPIC,
                live_qos,
            )
            if publish_two_drone
            else None
        )
        self._target_odom_pub = (
            node.create_publisher(Odometry, TARGET_ODOM_TOPIC, live_qos)
            if publish_two_drone
            else None
        )
        self._tf_pub = node.create_publisher(TFMessage, TF_TOPIC, live_qos)
        self._site_transforms = rotor_site_transforms_from_quad(quad)
        self._camera_transforms = (
            two_drone_camera_transforms_from_quad(quad)
            if publish_two_drone
            else ()
        )

    def publish(
        self,
        stamp: Any,
        chaser_state: Any,
        target_state: Any | None,
    ) -> tuple[GroundTruthOdomFields, GroundTruthOdomFields | None]:
        chaser_fields = ground_truth_odom_fields(chaser_state)
        target_fields = (
            ground_truth_odom_fields(target_state)
            if target_state is not None
            else None
        )
        if self._publish_two_drone and target_fields is None:
            raise ValueError("target state is required in two-drone telemetry mode")

        # Keep the pre-SCN names and frames as aliases while adding canonical
        # per-drone namespaced output.
        self._ground_truth_pub.publish(
            self.build_odometry(stamp, chaser_fields, BASE_LINK_FRAME)
        )
        if self._chaser_ground_truth_pub is not None:
            self._chaser_ground_truth_pub.publish(
                self.build_odometry(stamp, chaser_fields, CHASER_BASE_LINK_FRAME)
            )
        if self._target_odom_pub is not None and target_fields is not None:
            self._target_odom_pub.publish(
                self.build_odometry(stamp, target_fields, TARGET_BASE_LINK_FRAME)
            )
        self._tf_pub.publish(
            self.build_tf_message(stamp, chaser_fields, target_fields)
        )
        return chaser_fields, target_fields

    def build_odometry(
        self,
        stamp: Any,
        fields: GroundTruthOdomFields,
        child_frame_id: str,
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

    def build_tf_message(
        self,
        stamp: Any,
        chaser_fields: GroundTruthOdomFields,
        target_fields: GroundTruthOdomFields | None,
    ) -> Any:
        transforms = [
            self._build_transform(
                stamp,
                WORLD_FRAME,
                ODOM_FRAME,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            ),
            self._build_transform(
                stamp,
                WORLD_FRAME,
                BASE_LINK_FRAME,
                chaser_fields.position_enu,
                chaser_fields.orientation_xyzw,
            ),
        ]
        if self._publish_two_drone:
            transforms.append(
                self._build_transform(
                    stamp,
                    WORLD_FRAME,
                    CHASER_BASE_LINK_FRAME,
                    chaser_fields.position_enu,
                    chaser_fields.orientation_xyzw,
                )
            )
        if target_fields is not None:
            transforms.append(
                self._build_transform(
                    stamp,
                    WORLD_FRAME,
                    TARGET_BASE_LINK_FRAME,
                    target_fields.position_enu,
                    target_fields.orientation_xyzw,
                )
            )
        transforms.extend(
            self._build_transform(
                stamp,
                transform.parent_frame_id,
                transform.child_frame_id,
                transform.translation_xyz,
                transform.rotation_xyzw,
            )
            for transform in self._site_transforms
        )
        transforms.extend(
            self._build_transform(
                stamp,
                transform.parent_frame_id,
                transform.child_frame_id,
                transform.translation_xyz,
                transform.rotation_xyzw,
            )
            for transform in self._camera_transforms
        )
        message = self._tf_message_type()
        message.transforms = transforms
        return message

    @staticmethod
    def _build_transform(
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
