from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="tf_demos",
                executable="static_tf_broadcaster",
                name="static_tf_broadcaster",
                output="screen",
            ),
            Node(
                package="tf_demos",
                executable="can_link_broadcaster",
                name="can_link_broadcaster",
                output="screen",
            ),
        ]
    )
