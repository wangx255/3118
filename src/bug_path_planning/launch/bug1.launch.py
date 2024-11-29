from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ! PARAMS
    declare_initial_x = DeclareLaunchArgument(
        "initial_x",
        default_value="0.0",
        description="Initial X pose.",
    )
    declare_initial_y = DeclareLaunchArgument(
        "initial_y",
        default_value="8.0",
        description="Initial Y pose.",
    )
    declare_des_pos_x = DeclareLaunchArgument(
        "des_pos_x",
        default_value="2.0",
        description="Destination X pose.",
    )
    declare_des_pos_y = DeclareLaunchArgument(
        "des_pos_y",
        default_value="-3.0",
        description="Destination Y pose.",
    )

    # ! NODES
    boundary_following_node = Node(
        package="bug_path_planning",
        executable="boundary_following",
        name="boundary_following_node",
        output="screen",
    )

    motion_to_goal_node = Node(
        package="bug_path_planning",
        executable="motion_to_goal",
        name="motion_to_goal_node",
        output="screen",
        parameters=[
            {
                "des_pos_x": LaunchConfiguration("des_pos_x"),
                "des_pos_y": LaunchConfiguration("des_pos_y"),
            }
        ],
    )

    bug1_node = Node(
        package="bug_path_planning",
        executable="bug1",
        name="bug1_node",
        output="screen",
        parameters=[
            {
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "des_pos_x": LaunchConfiguration("des_pos_x"),
                "des_pos_y": LaunchConfiguration("des_pos_y"),
            }
        ],
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! PARAMETERS
    ld.add_action(declare_initial_x)
    ld.add_action(declare_initial_y)
    ld.add_action(declare_des_pos_x)
    ld.add_action(declare_des_pos_y)

    # ! NODES
    ld.add_action(boundary_following_node)
    ld.add_action(motion_to_goal_node)
    ld.add_action(bug1_node)

    return ld
