from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # ! PACKAGES DIR
    grid_planners_demo_dir = FindPackageShare(package="grid_planners_demo").find(
        "grid_planners_demo"
    )

    # ! NODES
    navigator_node = Node(
        package="grid_planners_demo",
        executable="navigator_dstar_lite",
        name="navigator_node",
        output="screen",
        parameters=[grid_planners_demo_dir + "/config/dstar_lite_params.yaml"],
    )

    path_follower_node = Node(
        package="path_follower",
        executable="path_follower",
        name="path_follower_node",
        output="screen",
        parameters=[{"linear_velocity": 0.12, "angular_velocity": 0.8}],
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! NODES
    ld.add_action(navigator_node)
    ld.add_action(path_follower_node)

    return ld
