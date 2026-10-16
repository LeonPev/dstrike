"""Frame conversions between MAVLink NED/FRD and ROS ENU/FLU."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


SQRT_HALF = math.sqrt(0.5)
EARTH_RADIUS_M = 6_378_137.0

# Both rotations are their own inverse up to quaternion sign, so the same
# sandwich converts ENU/FLU -> NED/FRD and NED/FRD -> ENU/FLU.
ENU_TO_NED_QUATERNION_WXYZ = (0.0, SQRT_HALF, SQRT_HALF, 0.0)
FRD_TO_FLU_QUATERNION_WXYZ = (0.0, 1.0, 0.0, 0.0)


@dataclass(frozen=True)
class LocalPositionNed:
    """MAVLink LOCAL_POSITION_NED sample in meters and meters/second."""

    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float


@dataclass(frozen=True)
class GlobalPositionInt:
    """MAVLink GLOBAL_POSITION_INT sample."""

    lat_e7: int
    lon_e7: int
    relative_alt_mm: int
    vx_cm_s: int
    vy_cm_s: int
    vz_cm_s: int


@dataclass(frozen=True)
class GlobalPositionOrigin:
    """Local tangent-plane origin for GLOBAL_POSITION_INT fallback odometry."""

    lat_deg: float
    lon_deg: float


@dataclass(frozen=True)
class RosOdomFields:
    """ROS odometry fields after converting to ENU/z-up conventions."""

    position_enu: tuple[float, float, float]
    linear_velocity_enu: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


def ned_to_enu_vector(vector: Iterable[float]) -> tuple[float, float, float]:
    """Convert an earth-frame vector from north/east/down to east/north/up."""
    north, east, down = _vector3(vector)
    return (east, north, -down)


def enu_to_ned_vector(vector: Iterable[float]) -> tuple[float, float, float]:
    """Convert an earth-frame vector from east/north/up to north/east/down."""
    east, north, up = _vector3(vector)
    return (north, east, -up)


def ros_enu_twist_to_mavlink_ned_velocity(
    linear_enu: Iterable[float],
) -> tuple[float, float, float]:
    """Convert ROS ENU linear velocity into MAVLink LOCAL_NED velocity."""
    return enu_to_ned_vector(linear_enu)


def ros_yaw_rate_to_mavlink_ned(yaw_rate_enu_rad_s: float) -> float:
    """Convert ROS z-up positive yaw rate to MAVLink NED positive yaw rate."""
    return -float(yaw_rate_enu_rad_s)


def attitude_rpy_to_quaternion_wxyz(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    """Build a scalar-first MAVLink/NED quaternion from roll, pitch, yaw."""
    cr = math.cos(float(roll_rad) * 0.5)
    sr = math.sin(float(roll_rad) * 0.5)
    cp = math.cos(float(pitch_rad) * 0.5)
    sp = math.sin(float(pitch_rad) * 0.5)
    cy = math.cos(float(yaw_rad) * 0.5)
    sy = math.sin(float(yaw_rad) * 0.5)

    return _normalise_quaternion_wxyz(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    )


def ned_frd_to_enu_flu_quaternion_xyzw(
    quaternion_wxyz: Iterable[float],
) -> tuple[float, float, float, float]:
    """Convert body-to-world orientation from MAVLink NED/FRD to ROS ENU/FLU."""
    q_ned_frd = _normalise_quaternion_wxyz(quaternion_wxyz)
    q_enu_flu = _quaternion_multiply_wxyz(
        ENU_TO_NED_QUATERNION_WXYZ,
        _quaternion_multiply_wxyz(q_ned_frd, FRD_TO_FLU_QUATERNION_WXYZ),
    )
    q_enu_flu = _canonical_quaternion_wxyz(_normalise_quaternion_wxyz(q_enu_flu))
    w, x, y, z = q_enu_flu
    return (x, y, z, w)


def local_position_ned_to_ros_odom_fields(
    local_position: LocalPositionNed,
    attitude_quaternion_wxyz: Iterable[float] | None = None,
) -> RosOdomFields:
    """Convert MAVLink local position and attitude into ROS odometry fields."""
    if attitude_quaternion_wxyz is None:
        attitude_quaternion_wxyz = (1.0, 0.0, 0.0, 0.0)

    return RosOdomFields(
        position_enu=ned_to_enu_vector(
            (local_position.x, local_position.y, local_position.z)
        ),
        linear_velocity_enu=ned_to_enu_vector(
            (local_position.vx, local_position.vy, local_position.vz)
        ),
        orientation_xyzw=ned_frd_to_enu_flu_quaternion_xyzw(
            attitude_quaternion_wxyz
        ),
    )


def global_position_int_to_local_ned(
    global_position: GlobalPositionInt,
    origin: GlobalPositionOrigin | None = None,
) -> tuple[LocalPositionNed, GlobalPositionOrigin]:
    """Convert GLOBAL_POSITION_INT to local NED using the first fix as origin."""
    lat_deg = float(global_position.lat_e7) / 10_000_000.0
    lon_deg = float(global_position.lon_e7) / 10_000_000.0
    if origin is None:
        origin = GlobalPositionOrigin(lat_deg=lat_deg, lon_deg=lon_deg)

    north_m = math.radians(lat_deg - origin.lat_deg) * EARTH_RADIUS_M
    east_m = (
        math.radians(lon_deg - origin.lon_deg)
        * EARTH_RADIUS_M
        * math.cos(math.radians(origin.lat_deg))
    )
    down_m = -float(global_position.relative_alt_mm) / 1000.0

    return (
        LocalPositionNed(
            x=north_m,
            y=east_m,
            z=down_m,
            vx=float(global_position.vx_cm_s) / 100.0,
            vy=float(global_position.vy_cm_s) / 100.0,
            vz=float(global_position.vz_cm_s) / 100.0,
        ),
        origin,
    )


def _vector3(vector: Iterable[float]) -> tuple[float, float, float]:
    values = tuple(float(value) for value in vector)
    if len(values) != 3:
        raise ValueError("vector must contain exactly three values")
    return values


def _normalise_quaternion_wxyz(
    quaternion: Iterable[float],
) -> tuple[float, float, float, float]:
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4:
        raise ValueError("quaternion must contain exactly four values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise ValueError("quaternion must have non-zero length")
    return tuple(value / norm for value in values)


def _canonical_quaternion_wxyz(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    if quaternion[0] >= 0.0:
        return quaternion
    return tuple(-value for value in quaternion)


def _quaternion_multiply_wxyz(
    left: Iterable[float],
    right: Iterable[float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = _normalise_quaternion_wxyz(left)
    rw, rx, ry, rz = _normalise_quaternion_wxyz(right)
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )
