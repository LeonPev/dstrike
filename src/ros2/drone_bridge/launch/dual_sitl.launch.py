"""Launch two isolated MAVLink adapters for SCN-010."""

from launch import LaunchDescription
from launch_ros.actions import Node

from drone_bridge.multi_vehicle_bridge import validate_routes


def generate_launch_description():
    nodes = []
    for route in validate_routes():
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
    return LaunchDescription(nodes)
