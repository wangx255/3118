from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():

    # ! PACKAGES DIR
    slam_examples_dir = FindPackageShare(package="slam_examples").find("slam_examples")

    # ! PARAMETERS
    declare_rviz_ver = DeclareLaunchArgument(
        name="rviz_ver",
        default_value="",
        description="RViz version config.",
    )

    # ! NDOES

    nav_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            slam_examples_dir + "/rviz/nav_demo.rviz",
        ],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("rviz_ver"), '" == "nav"'])
        ),
    )
    slam_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            slam_examples_dir + "/rviz/slam_demo.rviz",
        ],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("rviz_ver"), '" == "slam"'])
        ),
    )

    # ! EXTERNAL LAUNCH FILES

    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_examples_dir, "/launch/", "turtlebot3_house.launch.py"]
        ),
        launch_arguments={"x_pose": "-3", "y_pose": "1"}.items(),
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_examples_dir, "/launch/", "amcl.launch.py"]
        ),
        launch_arguments={"initial_pose_x": "-3.0", "initial_pose_y": "1.0"}.items(),
    )

    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [slam_examples_dir, "/launch/", "octomap_offline.launch.py"]
        ),
    )

    # ! NODES
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"yaml_filename": slam_examples_dir + "/maps/mymap.yaml"},
        ],
    )

    lifecycle_nodes = ["map_server", "amcl"]
    use_sim_time = True
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

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    ld.add_action(declare_rviz_ver)

    # ! NODES
    ld.add_action(nav_rviz_node)
    ld.add_action(slam_rviz_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)

    # ! LAUNCH FILES
    ld.add_action(tb3_launch)
    ld.add_action(amcl_launch)
    ld.add_action(octomap_launch)

    return ld
