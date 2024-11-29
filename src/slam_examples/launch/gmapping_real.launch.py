from launch import LaunchDescription
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="slam_gmapping",
                executable="slam_gmapping",
                output="screen",
                name="turtlebot3_slam_gmapping",
                parameters=[slam_examples_dir + "/config/gmapping_real_params.yaml"],
            ),
        ]
    )
