from launch import LaunchDescription
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                parameters=[
                    {"frame_id": "map", "use_height_map": True},
                    slam_examples_dir + "/config/octomap_params.yaml",
                ],
                remappings=[("/cloud_in", "/depth_camera/depth/points")],
            )
        ]
    )
