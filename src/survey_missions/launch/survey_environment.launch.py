from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    waypoint_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value='config/survey_waypoints_x.yaml', 
        description='Path to the waypoint YAML file'
    )


    mission_handler_node = Node(
        package='survey_missions',
        executable='mission_handler',
        name='mission_handler',
        output='screen',
        parameters=[{
            'waypoint_file': LaunchConfiguration('waypoint_file')  
        }]
    )

    return LaunchDescription([
        waypoint_file,  
        mission_handler_node  
    ])
