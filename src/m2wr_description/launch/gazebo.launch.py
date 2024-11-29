from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    # ! PACKAGE DIR
    m2wr_description_dir = FindPackageShare(package="m2wr_description").find(
        "m2wr_description"
    )

    gazebo_ros_dir = FindPackageShare(package="gazebo_ros").find("gazebo_ros")

    # ! ARGUMENTS
    world_arg = DeclareLaunchArgument(
        "world", default_value="sphere_example.world", description="World file name."
    )

    # ! LAUNCHES
    # Include the Gazebo empty_world.launch.py
    empty_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    gazebo_ros_dir,
                    "launch",
                    "gazebo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [m2wr_description_dir, "worlds", LaunchConfiguration("world")]
            ),
            "use_sim_time": "true",
        }.items(),
    )

    # Include the spawn.launch.py
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    m2wr_description_dir,
                    "launch",
                    "spawn.launch.py",
                ]
            )
        ),
        launch_arguments={"y": "2"}.items(),
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        PathJoinSubstitution(
                            [
                                m2wr_description_dir,
                                "urdf",
                                "m2wr_manual.xacro",
                            ]
                        ),
                    ]
                )
            }
        ],
    )

    # tf broadcaster node
    tf_broadcaster_node = Node(
        package="m2wr_description",
        executable="tf_broadcaster",
        name="chassis_tf_broadcaster",
    )

    # ? ================================

    ld = LaunchDescription()

    # !ARGUMENTS
    ld.add_action(world_arg)

    # ! NODES
    ld.add_action(robot_state_publisher_node)
    ld.add_action(tf_broadcaster_node)

    # ! LAUNCHES
    ld.add_action(empty_world_launch)
    ld.add_action(spawn_launch)

    return ld
