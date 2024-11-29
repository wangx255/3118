from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("turtlebot3_gazebo"),
                        "/launch",
                        "/turtlebot3_house.launch.py",
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("tf_demos"),
                        "/turtlebot3_slam.launch.py",
                    ]
                ),
            ),
        ]
    )
