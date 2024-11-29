from launch import LaunchDescription
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")

    # ! NODES
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"yaml_filename": slam_examples_dir + "/maps/map_real.yaml"},
        ],
    )

    lifecycle_nodes = ["map_server", "amcl"]
    use_sim_time = False
    autostart = True

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

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
                    slam_examples_dir + "/config/amcl_real_params.yaml",
                    {
                        "initial_pose.x": LaunchConfiguration("initial_pose_x"),
                        "initial_pose.y": LaunchConfiguration("initial_pose_y"),
                        "initial_pose.yaw": LaunchConfiguration("initial_pose_yaw"),
                    },
                ],
            ),
            map_server_node,
            lifecycle_manager_node,
        ]
    )
