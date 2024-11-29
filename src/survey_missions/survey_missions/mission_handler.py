import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import subprocess

class MissionHandler(Node):
    def __init__(self):
        super().__init__('mission_handler')

        # Parameters
        self.declare_parameter('waypoint_file', 'config/survey_waypoints_x.yaml')
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        # Load waypoints
        self.waypoints = self.load_waypoints(waypoint_file)
        self.current_waypoint_index = 0
        self.goal_reached = False

        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribers
        self.goal_reached_subscriber = self.create_subscription(
            Bool,
            '/goal_reached',
            self.goal_reached_callback,
            10
        )


        self.send_next_waypoint()

    def load_waypoints(self, filepath):
        package_path = get_package_share_directory('survey_missions')
        full_path = os.path.join(package_path, filepath)
        try:
            with open(full_path, 'r') as file:
                data = yaml.safe_load(file)
                if 'waypoints' not in data or not data['waypoints']:
                    raise ValueError('Waypoints list is empty')
                return data['waypoints']
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def goal_reached_callback(self, msg):
        if msg.data:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached.')
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.send_next_waypoint()
            else:
                self.save_map()

    def send_next_waypoint(self):
        waypoint = self.waypoints[self.current_waypoint_index]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = waypoint['x']
        goal.pose.position.y = waypoint['y']
        goal.pose.position.z = waypoint['z']
        goal.pose.orientation.w = 1.0  
        self.goal_publisher.publish(goal)
        self.get_logger().info(f'Sent waypoint {self.current_waypoint_index}: {waypoint}')

    def save_map(self):
        try:
                
            package_share_directory = get_package_share_directory('survey_missions')
            maps_directory = os.path.join(package_share_directory, 'maps')

            if not os.path.exists(maps_directory):
                os.makedirs(maps_directory)

            existing_maps = [f for f in os.listdir(maps_directory) if f.startswith("mymap") and f.endswith(".yaml")]
            map_number = len(existing_maps) + 1
            map_filename = f"mymap{map_number}"
            map_filepath = os.path.join(maps_directory, map_filename)
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_filepath],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            if result.returncode == 0:
                self.get_logger().info(f'Map saved successfully: {map_filepath}')
            else:
                self.get_logger().error(f'Failed to save map: {result.stderr.decode()}')

        except Exception as e:
            self.get_logger().error(f'Error while saving map: {e}')



def main(args=None):
    rclpy.init(args=args)
    mission_handler = MissionHandler()
    rclpy.spin(mission_handler)
    mission_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
