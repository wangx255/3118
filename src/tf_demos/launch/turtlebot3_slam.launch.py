from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.conditions import IfCondition


def generate_launch_description():

    launch_description = LaunchDescription()

    # node configs
    rviz_gui = DeclareLaunchArgument(
        "rviz_gui", default_value=TextSubstitution(text="true")
    )

    # dir configs
    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("tf_demos"), "tf_demo.rviz"]
    )

    # add parameters
    launch_description.add_action(rviz_gui)

    # ! LAUNCHES
    slam_gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("slam_gmapping"),
                "/launch",
                "/slam_gmapping.launch.py",
            ]
        ),
    )

    # ! NODES
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz_gui")),
    )

    launch_description.add_action(slam_gmapping_launch)
    launch_description.add_action(rviz_node)

    return launch_description
