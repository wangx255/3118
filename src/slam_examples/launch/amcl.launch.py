from launch import LaunchDescription
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="initial_pose_x",
                default_value="0.0",
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="initial_pose_y",
                default_value="0.0",
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="initial_pose_yaw",
                default_value="0.0",
                description="Absolute path to rviz config file",
            ),
            launch_ros.actions.Node(
                package="nav2_amcl",
                executable="amcl",
                output="screen",
                name="amcl",
                parameters=[
                    slam_examples_dir + "/config/amcl_params.yaml",
                    {
                        "initial_pose.x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose.y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose.yaw": LaunchConfiguration("initial_pose_yaw"),
                    },
                ],
            ),
        ]
    )
