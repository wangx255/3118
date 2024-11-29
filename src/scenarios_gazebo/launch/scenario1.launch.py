import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ! PACKAGES DIR
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    scenarios_gazebo_dir = FindPackageShare(package="scenarios_gazebo").find(
        "scenarios_gazebo"
    )

    # ! Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": scenarios_gazebo_dir + "/worlds/world02.world"
        }.items(),
    )

    tb3_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scenarios_gazebo_dir, "launch", "tb3_spawn.launch.py")
        ),
        launch_arguments={"y": "7.5"}.items(),
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! NODES
    ld.add_action(start_gazebo_server_cmd)

    # ! LAUNCHES
    ld.add_action(tb3_spawn_launch)

    return ld
