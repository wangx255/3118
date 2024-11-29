from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                name="turtlesim_node",
            ),
            Node(
                package="beginner_tutorials",
                executable="velocity_pub_py",
                name="velocity_pub_node",
            ),
            Node(
                package="beginner_tutorials",
                executable="velocity_sub_py",
                name="velocity_sub_node",
            ),
        ]
    )
