"""Launch two isolated MAVLink adapters for SCN-010."""

from launch import LaunchDescription
from launch_ros.actions import Node

from drone_bridge.multi_vehicle_bridge import validate_routes


def generate_launch_description():
    nodes = []
    routes = validate_routes()
    for route in routes:
        nodes.append(Node(
            package="drone_bridge",
            executable="mavlink_bridge",
            namespace=route.namespace,
            name="mavlink_bridge",
            parameters=[{
                "connect": route.connect,
                "use_sim_state": True,
                "odom_frame_id": "world",
                "odom_alignment_topic": route.odom_alignment_topic,
                "child_frame_id": route.child_frame_id,
            }],
        ))
    target_route = next(route for route in routes if route.role == "target")
    nodes.append(Node(
        package="drone_bridge",
        executable="target_route_controller",
        namespace=target_route.namespace,
        name="target_route_controller",
    ))
    return LaunchDescription(nodes)
