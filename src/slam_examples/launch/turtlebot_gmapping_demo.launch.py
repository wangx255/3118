from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ! PACKAGES DIR
    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")
    turtlebot3_gazebo_dir = FindPackageShare(package="turtlebot3_gazebo").find(
        "turtlebot3_gazebo"
    )

    rviz_node_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", slam_examples_dir + "/rviz/slam_demo.rviz"],
    )

    # ! EXTERNAL LAUNCH FILES

    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [turtlebot3_gazebo_dir, "/launch/", "turtlebot3_house.launch.py"]
        ),
        launch_arguments={"x_pose": "-3", "y_pose": "1"}.items(),
    )

    gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_examples_dir, "/launch/", "gmapping.launch.py"]
        )
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! NODES
    ld.add_action(rviz_node_cmd)

    # ! LAUNCH FILES
    ld.add_action(tb3_launch)
    ld.add_action(gmapping_launch)

    return ld
