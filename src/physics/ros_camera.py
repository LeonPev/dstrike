"""ROS 2 image publishing for the MuJoCo front camera."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import time
from typing import Any, Iterable, Mapping


DEFAULT_CAMERA_NAME = "front_cam"
DEFAULT_CAMERA_FRAME = "front_cam"
DEFAULT_IMAGE_TOPIC = "/sim/camera/front/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/sim/camera/front/camera_info"
DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480
DEFAULT_CAMERA_FPS = 10.0

TARGET_CAMERA_NAME = "target_front_cam"
CHASER_CAMERA_NAME = "chaser_front_cam"
TARGET_CAMERA_FRAME = "target/front_camera"
CHASER_CAMERA_FRAME = "chaser/front_camera"
TARGET_IMAGE_TOPIC = "/sim/target/camera/front/image_raw"
TARGET_CAMERA_INFO_TOPIC = "/sim/target/camera/front/camera_info"
CHASER_IMAGE_TOPIC = "/sim/chaser/camera/front/image_raw"
CHASER_CAMERA_INFO_TOPIC = "/sim/chaser/camera/front/camera_info"


@dataclass(frozen=True)
class CameraConfig:
    enabled: bool = True
    camera_name: str = DEFAULT_CAMERA_NAME
    frame_id: str = DEFAULT_CAMERA_FRAME
    image_topic: str = DEFAULT_IMAGE_TOPIC
    camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC
    width: int = DEFAULT_CAMERA_WIDTH
    height: int = DEFAULT_CAMERA_HEIGHT
    fps: float = DEFAULT_CAMERA_FPS

    @classmethod
    def from_env(cls) -> "CameraConfig":
        return cls.from_mapping(os.environ)

    @classmethod
    def from_mapping(cls, values: Mapping[str, str]) -> "CameraConfig":
        enabled = parse_bool(values.get("CAMERA_ENABLED", "true"))
        if not enabled:
            return cls(enabled=False)

        return cls(
            enabled=enabled,
            camera_name=values.get("CAMERA_NAME", DEFAULT_CAMERA_NAME),
            frame_id=values.get("CAMERA_FRAME", DEFAULT_CAMERA_FRAME),
            image_topic=values.get("CAMERA_TOPIC", DEFAULT_IMAGE_TOPIC),
            camera_info_topic=values.get(
                "CAMERA_INFO_TOPIC", DEFAULT_CAMERA_INFO_TOPIC
            ),
            width=parse_positive_int(
                values.get("CAMERA_WIDTH", str(DEFAULT_CAMERA_WIDTH)),
                "CAMERA_WIDTH",
            ),
            height=parse_positive_int(
                values.get("CAMERA_HEIGHT", str(DEFAULT_CAMERA_HEIGHT)),
                "CAMERA_HEIGHT",
            ),
            fps=parse_positive_float(
                values.get("CAMERA_FPS", str(DEFAULT_CAMERA_FPS)),
                "CAMERA_FPS",
            ),
        )


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def camera_configs_for_scenario(
    scenario_name: str,
    *,
    camera_rate_hz: float | None = None,
    values: Mapping[str, str] | None = None,
) -> tuple[CameraConfig, ...]:
    """Resolve the legacy or two-drone camera contract from one env switch."""
    source = os.environ if values is None else values
    legacy = CameraConfig.from_mapping(source)
    if not legacy.enabled:
        return ()
    if scenario_name != "mountain_two_drones":
        return (legacy,)
    if camera_rate_hz is None or camera_rate_hz <= 0.0:
        raise ValueError("two-drone camera rate must be positive")

    common = {
        "enabled": True,
        "width": legacy.width,
        "height": legacy.height,
        "fps": camera_rate_hz,
    }
    return (
        CameraConfig(
            camera_name=TARGET_CAMERA_NAME,
            frame_id=TARGET_CAMERA_FRAME,
            image_topic=TARGET_IMAGE_TOPIC,
            camera_info_topic=TARGET_CAMERA_INFO_TOPIC,
            **common,
        ),
        CameraConfig(
            camera_name=CHASER_CAMERA_NAME,
            frame_id=CHASER_CAMERA_FRAME,
            image_topic=CHASER_IMAGE_TOPIC,
            camera_info_topic=CHASER_CAMERA_INFO_TOPIC,
            **common,
        ),
    )


def ros_image_pixels(pixels: Any) -> Any:
    """Convert the active MuJoCo GL buffer to ROS top-left image row order."""
    import numpy as np

    rgb = np.asarray(pixels, dtype=np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"camera pixels must have HxWx3 shape, got {rgb.shape}")
    # mujoco.Renderer already normalizes EGL/OSMesa buffers to top-left row
    # order. ROS only needs a contiguous RGB buffer.
    return np.ascontiguousarray(rgb)


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"expected boolean value, got {value!r}")


def parse_positive_int(value: str, name: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if parsed <= 0:
        raise ValueError(f"{name} must be positive")
    return parsed


def parse_positive_float(value: str, name: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be a number") from exc
    if parsed <= 0.0:
        raise ValueError(f"{name} must be positive")
    return parsed


def camera_intrinsics_from_fovy(
    width: int,
    height: int,
    fovy_degrees: float,
) -> CameraIntrinsics:
    if width <= 0 or height <= 0:
        raise ValueError("camera width and height must be positive")
    if fovy_degrees <= 0.0 or fovy_degrees >= 180.0:
        raise ValueError("camera fovy must be between 0 and 180 degrees")

    fovy_radians = math.radians(fovy_degrees)
    focal_length = height / (2.0 * math.tan(fovy_radians / 2.0))
    return CameraIntrinsics(
        fx=focal_length,
        fy=focal_length,
        cx=(width - 1) / 2.0,
        cy=(height - 1) / 2.0,
    )


class SimCameraPublisher:
    """Render MuJoCo's front camera offscreen and publish ROS Image messages."""

    def __init__(
        self,
        quad: Any,
        config: CameraConfig | None = None,
    ) -> None:
        self._quad = quad
        self._config = config or CameraConfig.from_env()
        if not self._config.enabled:
            raise ValueError("SimCameraPublisher should not be created when disabled")

        import mujoco
        import rclpy
        from sensor_msgs.msg import CameraInfo, Image

        self._mujoco = mujoco
        self._rclpy = rclpy
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=None)

        self._camera_id = mujoco.mj_name2id(
            quad.model, mujoco.mjtObj.mjOBJ_CAMERA, self._config.camera_name
        )
        if self._camera_id < 0:
            raise ValueError(
                f"MuJoCo model must contain camera named {self._config.camera_name}"
            )

        quad.model.vis.global_.offwidth = self._config.width
        quad.model.vis.global_.offheight = self._config.height
        self._gl_context = mujoco.GLContext(self._config.width, self._config.height)
        self._gl_context.make_current()
        self._renderer = mujoco.Renderer(
            quad.model,
            height=self._config.height,
            width=self._config.width,
        )

        self._node = rclpy.create_node("sim_camera_publisher")
        self._image_type = Image
        self._camera_info_type = CameraInfo
        self._image_pub = self._node.create_publisher(
            Image,
            self._config.image_topic,
            10,
        )
        self._camera_info_pub = self._node.create_publisher(
            CameraInfo,
            self._config.camera_info_topic,
            10,
        )
        self._publish_period_s = 1.0 / self._config.fps
        self._last_publish_monotonic_s: float | None = None
        self._node.get_logger().info(
            "publishing MuJoCo camera topics "
            f"{self._config.image_topic}, {self._config.camera_info_topic} "
            f"camera={self._config.camera_name} "
            f"resolution={self._config.width}x{self._config.height} "
            f"fps={self._config.fps:g}"
        )

    def close(self) -> None:
        self._renderer.close()
        self._gl_context.free()
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()

    def maybe_publish(self, now_monotonic_s: float | None = None) -> bool:
        now_monotonic_s = (
            time.monotonic() if now_monotonic_s is None else now_monotonic_s
        )
        if (
            self._last_publish_monotonic_s is not None
            and now_monotonic_s - self._last_publish_monotonic_s
            < self._publish_period_s
        ):
            return False

        self.publish()
        self._last_publish_monotonic_s = now_monotonic_s
        return True

    def publish(self) -> None:
        self._gl_context.make_current()
        self._renderer.update_scene(
            self._quad.data,
            camera=self._config.camera_name,
        )
        pixels = self._renderer.render()
        stamp = self._node.get_clock().now().to_msg()
        self._image_pub.publish(self._build_image(stamp, pixels))
        self._camera_info_pub.publish(self._build_camera_info(stamp))
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def _build_image(self, stamp: Any, pixels: Any) -> Any:
        rgb = ros_image_pixels(pixels)
        expected_shape = (self._config.height, self._config.width, 3)
        if rgb.shape != expected_shape:
            raise RuntimeError(
                f"unexpected camera render shape {rgb.shape}, expected {expected_shape}"
            )

        image = self._image_type()
        image.header.stamp = stamp
        image.header.frame_id = self._config.frame_id
        image.height = self._config.height
        image.width = self._config.width
        image.encoding = "rgb8"
        image.is_bigendian = 0
        image.step = self._config.width * 3
        image.data = rgb.tobytes()
        return image

    def _build_camera_info(self, stamp: Any) -> Any:
        info = self._camera_info_type()
        info.header.stamp = stamp
        info.header.frame_id = self._config.frame_id
        info.height = self._config.height
        info.width = self._config.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        intrinsics = camera_intrinsics_from_fovy(
            width=self._config.width,
            height=self._config.height,
            fovy_degrees=float(self._quad.model.cam_fovy[self._camera_id]),
        )
        info.k = [
            intrinsics.fx,
            0.0,
            intrinsics.cx,
            0.0,
            intrinsics.fy,
            intrinsics.cy,
            0.0,
            0.0,
            1.0,
        ]
        info.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        info.p = [
            intrinsics.fx,
            0.0,
            intrinsics.cx,
            0.0,
            0.0,
            intrinsics.fy,
            intrinsics.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return info


class SimCameraPublishers:
    """Thin multi-camera coordinator around the existing publisher."""

    def __init__(self, quad: Any, configs: Iterable[CameraConfig]) -> None:
        self._publishers = tuple(
            SimCameraPublisher(quad, config) for config in configs
        )

    def close(self) -> None:
        # The first publisher owns rclpy; close it last.
        for publisher in reversed(self._publishers):
            publisher.close()

    def publish(self) -> None:
        for publisher in self._publishers:
            publisher.publish()

    def maybe_publish(self, now_monotonic_s: float | None = None) -> bool:
        published = False
        for publisher in self._publishers:
            published = publisher.maybe_publish(now_monotonic_s) or published
        return published
