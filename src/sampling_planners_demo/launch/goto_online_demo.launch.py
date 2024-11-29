from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # ! PACKAGES DIR
    sampling_planners_demo_dir = FindPackageShare(
        package="sampling_planners_demo"
    ).find("sampling_planners_demo")

    # ! NODES
    navigator_node = Node(
        package="sampling_planners_demo",
        executable="online_navigator.py",  # ? CONFIGURED NODE EXECUTABLE
        name="online_navigator_node",  # ? CONFIGURED NODE NAME
        output="screen",
        parameters=[
            sampling_planners_demo_dir + "/config/online_params.yaml"
        ],  # ? CHANGED THE NAME OF THE PARAMS FILE NAME
    )

    path_follower_node = Node(
        package="path_follower",
        executable="path_follower",
        name="path_follower_node",
        output="screen",
        parameters=[
            {"linear_velocity": 0.15, "angular_velocity": 1.0}
        ],  # ? DECREASED MAXIMUM VELOCITIES SO THAT THE ROBOT
        # ? HAS LESS JERKY MOVEMENTS AND TRACK OF PATH WHEN REPLANNING
        condition=IfCondition(LaunchConfiguration("enable_move", default=True)),
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! NODES
    ld.add_action(navigator_node)
    ld.add_action(path_follower_node)

    return ld
