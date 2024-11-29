import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import subprocess
from math import sqrt
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

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

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Send the first waypoint
        if self.waypoints:
            self.send_next_waypoint()
        else:
            self.get_logger().error("No waypoints loaded. Stopping mission handler.")

    def load_waypoints(self, filepath):
        """Load waypoints from YAML file."""
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

    def is_goal_reached(self, robot_position, goal_position, tolerance=0.5):
        """Check if the robot is within the tolerance of the goal position."""
        distance = sqrt((robot_position[0] - goal_position[0])**2 + 
                        (robot_position[1] - goal_position[1])**2)
        return distance <= tolerance

    def get_robot_position(self):
        """Retrieve the robot's current position using TF."""
        try:
            now = self.get_clock().now()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            position = trans.transform.translation
            return [position.x, position.y, position.z]
        except Exception as e:
            self.get_logger().error(f'Failed to get robot position: {e}')
            return None

    def goal_reached_callback(self, msg):
        """Callback for goal_reached topic."""
        if msg.data:
            robot_position = self.get_robot_position()
            if robot_position is None:
                self.get_logger().warn("Robot position unavailable. Cannot verify goal reached.")
                return

            goal = self.waypoints[self.current_waypoint_index]
            goal_position = [
                goal['x'],
                goal['y'],
                goal.get('z', 0.0)
            ]
            goal_tolerance = goal.get('tolerance', 0.5)

            if self.is_goal_reached(robot_position, goal_position, tolerance=goal_tolerance):
                self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached.')
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    self.send_next_waypoint()
                else:
                    self.save_map()
            else:
                self.get_logger().warn(f"Robot not close enough to waypoint {self.current_waypoint_index}. Retrying...")
                self.retry_waypoint()

    def send_next_waypoint(self):
        """Send the next waypoint to the robot."""
        waypoint = self.waypoints[self.current_waypoint_index]

        # Check if the waypoint is valid
        if not self.is_point_valid(waypoint):  # Add your own logic for point validity
            self.get_logger().warn(f"Waypoint {self.current_waypoint_index} is invalid. Skipping...")
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.send_next_waypoint()
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = waypoint['x']
        goal.pose.position.y = waypoint['y']
        goal.pose.position.z = waypoint.get('z', 0.0)
        goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal)
        self.get_logger().info(f'Sent waypoint {self.current_waypoint_index}: {waypoint}')

    def retry_waypoint(self):
        """Retry the current waypoint."""
        self.get_logger().info(f"Retrying waypoint {self.current_waypoint_index}...")
        self.send_next_waypoint()

    def is_point_valid(self, waypoint):
        """Validate the waypoint."""
        # Add collision or feasibility checks here if necessary
        return True

    def save_map(self):
        """Save the current map."""
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

