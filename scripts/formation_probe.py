#!/usr/bin/env python3
"""Geometry and hold-window helpers shared by the SCN-009 live probe."""

from __future__ import annotations

import math


def distance(a, b) -> float:
    return math.dist(tuple(a), tuple(b))


def point_to_route_error(point, route_start, heading_rad: float) -> float:
    """Return XY cross-track distance from an infinite configured route line."""
    dx = point[0] - route_start[0]
    dy = point[1] - route_start[1]
    return abs(-math.sin(heading_rad) * dx + math.cos(heading_rad) * dy)


def body_offset_world(target, yaw_rad: float, offset):
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return (
        target[0] + cosine * offset[0] - sine * offset[1],
        target[1] + sine * offset[0] + cosine * offset[1],
        target[2] + offset[2],
    )


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )
