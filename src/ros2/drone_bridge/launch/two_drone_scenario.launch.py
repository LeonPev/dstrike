"""Launch the MAVLink adapter and two-drone scenario controller."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="drone_bridge", executable="mavlink_bridge", name="drone_mavlink_bridge", parameters=[{
            "connect": "udpin:0.0.0.0:14550",
            "use_sim_state": True,
            "odom_frame_id": "world",
            "odom_alignment_topic": "/sim/chaser/ground_truth/odom",
        }]),
        Node(package="drone_bridge", executable="scenario_controller", name="scenario_controller"),
    ])
