import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import SetBool
import math
import tf_transformations


class BugTangent(Node):
    def __init__(self):
        super().__init__("bug_tangent_node")

        self.yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees

        # Parameters
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("des_pos_x", 0.0)
        self.declare_parameter("des_pos_y", 0.0)

        # Initial position of robot
        self.position_ = Point()
        self.initial_position_ = Point()
        self.initial_position_.x = (
            self.get_parameter("initial_x").get_parameter_value().double_value
        )
        self.initial_position_.y = (
            self.get_parameter("initial_y").get_parameter_value().double_value
        )
        self.initial_position_.z = 0.0

        # Goal position of robot
        self.desired_position_ = Point()
        self.desired_position_.x = (
            self.get_parameter("des_pos_x").get_parameter_value().double_value
        )
        self.desired_position_.y = (
            self.get_parameter("des_pos_y").get_parameter_value().double_value
        )
        self.desired_position_.z = 0.0

        self.regions_ = None
        self.state_desc_ = ["Go to point.", "Boundary following.", "Cannot reach goal."]
        self.state_ = 0
        self.initial_distance_ = None

        self.sub_laser_ = self.create_subscription(
            LaserScan, "/scan", self.clbk_laser, 10
        )
        self.sub_odom_ = self.create_subscription(Odometry, "/odom", self.clbk_odom, 10)

        self.cli_motion_to_goal_ = self.create_client(SetBool, "/motion_to_goal_switch")
        self.cli_boundary_follower_ = self.create_client(
            SetBool, "/boundary_follower_switch"
        )
        self.cli_set_entity_state_ = self.create_client(
            SetEntityState, "/gazebo/set_entity_state"
        )

        while not self.cli_motion_to_goal_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "/motion_to_goal_switch service not available, waiting..."
            )

        while not self.cli_boundary_follower_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "/boundary_follower_switch service not available, waiting..."
            )

        while not self.cli_set_entity_state_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "/gazebo/set_entity_state service not available, waiting..."
            )

        self.initial_distance_ = math.sqrt(
            math.pow(self.position_.x - self.desired_position_.x, 2)
            + math.pow(self.position_.y - self.desired_position_.y, 2)
        )

        # Set robot position
        self.set_robot_position()

        # Initialize going to the point
        self.change_state(0)

        # Timer for the algorithm
        self.create_timer(1.0 / 20, self.main_callback)

    def clbk_odom(self, msg):
        # position
        self.position_ = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]

    def clbk_laser(self, msg):
        self.regions_ = {
            "right": min(min(msg.ranges[269:270]), 10),
            "fright": min(min(msg.ranges[314:315]), 10),
            "front": min(min(msg.ranges[0:1]), 10),
            "fleft": min(min(msg.ranges[44:45]), 10),
            "left": min(min(msg.ranges[89:90]), 10),
        }
        self.laser_data_ = msg.ranges

    def change_state(self, state):
        self.state_ = state
        log = "state changed: %s" % self.state_desc_[state]
        self.get_logger().info(log)
        if self.state_ == 0:
            self.cli_motion_to_goal_.call_async(SetBool.Request(data=True))
            self.cli_boundary_follower_.call_async(SetBool.Request(data=False))
        elif self.state_ == 1:
            self.cli_motion_to_goal_.call_async(SetBool.Request(data=False))
            self.cli_boundary_follower_.call_async(SetBool.Request(data=True))
        elif self.state_ == 2:
            self.cli_motion_to_goal_.call_async(SetBool.Request(data=False))
            self.cli_boundary_follower_.call_async(SetBool.Request(data=False))

    def normalize_angle(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def set_robot_position(self):
        entity_state = SetEntityState.Request()
        entity_state.state.name = "burger"
        entity_state.state.pose.position.x = self.initial_position_.x
        entity_state.state.pose.position.y = self.initial_position_.y
        self.cli_set_entity_state_.call_async(entity_state)

    def main_callback(self):
        if self.regions_ is None:
            return

        if self.state_ == 0:
            # MOVING TOWARDS THE GOAL
            if 0.15 < self.regions_["front"] < 1:
                curr_distance = math.sqrt(
                    math.pow(self.position_.x - self.desired_position_.x, 2)
                    + math.pow(self.position_.y - self.desired_position_.y, 2)
                )
                if self.initial_distance_ < curr_distance:
                    self.get_logger().warn("The goal is not reachable.")
                self.initial_distance_ = curr_distance
                self.change_state(1)

        elif self.state_ == 1:
            desired_yaw = math.atan2(
                self.desired_position_.y - self.position_.y,
                self.desired_position_.x - self.position_.x,
            )
            err_yaw = self.normalize_angle(desired_yaw - self.yaw_)

            if (
                math.fabs(err_yaw) < (math.pi / 6)
                and self.regions_["front"] > 1.5
                and self.regions_["fright"] > 1
                and self.regions_["fleft"] > 1
            ):
                self.get_logger().info("less than 30")
                self.change_state(0)

            if (
                err_yaw > 0
                and math.fabs(err_yaw) > (math.pi / 6)
                and math.fabs(err_yaw) < (math.pi / 2)
                and self.regions_["left"] > 1.5
                and self.regions_["fleft"] > 1
            ):
                self.get_logger().info("between 30 and 90 - to the left")
                self.change_state(0)

            if (
                err_yaw < 0
                and math.fabs(err_yaw) > (math.pi / 6)
                and math.fabs(err_yaw) < (math.pi / 2)
                and self.regions_["right"] > 1.5
                and self.regions_["fright"] > 1
            ):
                self.get_logger().info("between 30 and 90 - to the right")
                self.change_state(0)


def main(args=None):
    rclpy.init(args=args)
    bug_tangent_node = BugTangent()
    rclpy.spin(bug_tangent_node)

    bug_tangent_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
