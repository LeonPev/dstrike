"""Configuration contract for SCN-010's isolated MAVLink/ROS adapters."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleBridgeRoute:
    role: str
    namespace: str
    mavlink_port: int
    sysid: int
    odom_alignment_topic: str
    child_frame_id: str

    @property
    def connect(self) -> str:
        return f"udpin:0.0.0.0:{self.mavlink_port}"

    @property
    def heartbeat_topic(self) -> str:
        return f"/{self.namespace}/drone/heartbeat"

    @property
    def odom_topic(self) -> str:
        return f"/{self.namespace}/drone/odom"


DUAL_SITL_ROUTES = (
    VehicleBridgeRoute("target", "target", 14551, 2, "/sim/target/odom", "target/base_link"),
    VehicleBridgeRoute("chaser", "chaser", 14550, 1, "/sim/chaser/ground_truth/odom", "chaser/base_link"),
)


def validate_routes(routes=DUAL_SITL_ROUTES):
    routes = tuple(routes)
    if {route.role for route in routes} != {"target", "chaser"}:
        raise ValueError("dual SITL bridge requires target and chaser routes")
    for attribute in ("namespace", "mavlink_port", "sysid", "child_frame_id"):
        values = [getattr(route, attribute) for route in routes]
        if len(values) != len(set(values)):
            raise ValueError(f"vehicle bridge {attribute} values must be unique")
    return routes
