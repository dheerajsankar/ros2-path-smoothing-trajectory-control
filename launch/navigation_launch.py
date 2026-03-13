from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="navigation",
            executable="sim_robot",
        ),
        Node(
            package="navigation",
            executable="waypoints",
        ),
        Node(
            package="navigation",
            executable="path_smoothening",
        ),
        Node(
            package="navigation",
            executable="trajectory_gen",
        ),
        Node(
            package="navigation",
            executable="controller",
        ),
    ])  