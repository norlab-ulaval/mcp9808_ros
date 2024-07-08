from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mcp9808_ros",
                executable="mcp9808_node",
                name="mcp9808_node",
                parameters=[
                    {"pub_rate": 100},
                ],
            )
        ]
    )
