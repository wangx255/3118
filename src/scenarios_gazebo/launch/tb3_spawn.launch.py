import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    turtlebot3_gazebo_dir = FindPackageShare(package="turtlebot3_gazebo").find(
        "turtlebot3_gazebo"
    )

    # ! PARAMETERS
    x_arg = DeclareLaunchArgument("x", default_value="0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="0", description="Y position")
    z_arg = DeclareLaunchArgument("z", default_value="7.5", description="Z position")

    # ! LAUNCHES

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            turtlebot3_gazebo_dir + "/launch/robot_state_publisher.launch.py"
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            turtlebot3_gazebo_dir + "/launch/spawn_turtlebot3.launch.py"
        ),
        launch_arguments={
            "x_pose": LaunchConfiguration("x"),
            "y_pose": LaunchConfiguration("y"),
        }.items(),
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! DECLARE ARGUMENTS
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
