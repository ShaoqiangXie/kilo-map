from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="legkilo",
                executable="backend_submap_publisher",
                name="backend_submap_publisher",
                output="screen",
            ),
        ]
    )
