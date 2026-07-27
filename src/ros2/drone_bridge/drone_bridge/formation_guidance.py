"""Bounded target-body formation guidance without ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass
import math


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class GuidanceResult:
    desired_xyz: Vector3
    error_m: float
    velocity_xyz: Vector3


def target_body_point(
    target_xyz: Vector3,
    target_yaw_rad: float,
    offset_target_body_m: Vector3,
) -> Vector3:
    """Rotate a target-body FLU offset into the world ENU frame."""
    forward, left, up = offset_target_body_m
    cosine = math.cos(target_yaw_rad)
    sine = math.sin(target_yaw_rad)
    return (
        target_xyz[0] + cosine * forward - sine * left,
        target_xyz[1] + sine * forward + cosine * left,
        target_xyz[2] + up,
    )


def formation_command(
    chaser_xyz: Vector3,
    target_xyz: Vector3,
    target_yaw_rad: float,
    offset_target_body_m: Vector3,
    max_speed_mps: float,
    *,
    target_velocity_xyz: Vector3 = (0.0, 0.0, 0.0),
    position_gain: float = 1.0,
) -> GuidanceResult:
    """Return feed-forward plus proportional velocity, bounded in magnitude."""
    if max_speed_mps <= 0.0 or position_gain < 0.0:
        raise ValueError("guidance speed must be positive and gain non-negative")
    values = (*chaser_xyz, *target_xyz, target_yaw_rad, *offset_target_body_m, *target_velocity_xyz)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("guidance inputs must be finite")

    desired = target_body_point(target_xyz, target_yaw_rad, offset_target_body_m)
    delta = tuple(desired[index] - chaser_xyz[index] for index in range(3))
    error = math.sqrt(sum(component * component for component in delta))
    unconstrained = tuple(
        target_velocity_xyz[index] + position_gain * delta[index]
        for index in range(3)
    )
    speed = math.sqrt(sum(component * component for component in unconstrained))
    if speed > max_speed_mps:
        scale = max_speed_mps / speed
        velocity = tuple(component * scale for component in unconstrained)
    else:
        velocity = unconstrained
    return GuidanceResult(desired, error, velocity)
