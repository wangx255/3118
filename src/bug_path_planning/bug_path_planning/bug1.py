import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import SetBool

import math


class Bug1(Node):
    def __init__(self):
        super().__init__("bug1_node")

        self.yaw_ = 0
        self.yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees

        # Parameters
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("des_pos_x", 0.0)
        self.declare_parameter("des_pos_y", 0.0)

        # INITIAL POSITION OF ROBOT
        self.position_ = Point()
        self.initial_position_ = Point()
        self.initial_position_.x = (
            self.get_parameter("initial_x").get_parameter_value().double_value
        )
        self.initial_position_.y = (
            self.get_parameter("initial_y").get_parameter_value().double_value
        )
        self.initial_position_.z = 0.0

        # GOAL POSITION OF ROBOT
        self.desired_position_ = Point()
        self.desired_position_.x = (
            self.get_parameter("des_pos_x").get_parameter_value().double_value
        )
        self.desired_position_.y = (
            self.get_parameter("des_pos_y").get_parameter_value().double_value
        )
        self.desired_position_.z = 0.0

        self.regions_ = None
        self.state_desc_ = [
            "Go to point",
            "circumnavigate obstacle",
            "go to closest point",
        ]
        self.state_ = 0
        self.circumnavigate_starting_point_ = Point()
        self.circumnavigate_closest_point_ = Point()
        self.count_state_time_ = 0  # seconds the robot is in a state
        self.count_loop_ = 0
        self.laser_data_ = []

        # Subscribers
        self.sub_laser_ = self.create_subscription(
            LaserScan, "/scan", self.clbk_laser, 10
        )
        self.sub_odom_ = self.create_subscription(Odometry, "/odom", self.clbk_odom, 10)

        # Service clients
        self.cli_motion_to_goal_ = self.create_client(SetBool, "/motion_to_goal_switch")
        self.cli_boundary_follower_ = self.create_client(
            SetBool, "/boundary_follower_switch"
        )
        self.cli_set_entity_state_ = self.create_client(
            SetEntityState, "/gazebo/set_entity_state"
        )

        # Set initial robot position
        self.set_robot_position()

        # Initialize going to the point
        self.change_state(0)

        # Timer for main loop
        self.timer_ = self.create_timer(0.05, self.update)

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
        euler = euler_from_quaternion(quaternion)
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
        self.count_state_time_ = 0
        self.state_ = state
        log = "state changed: %s" % self.state_desc_[state]
        self.get_logger().info(log)
        if self.state_ == 0:
            self.cli_motion_to_goal_.call_async(SetBool.Request(data=True))
            self.cli_boundary_follower_.call_async(SetBool.Request(data=False))
        elif self.state_ == 1 or self.state_ == 2:
            self.cli_motion_to_goal_.call_async(SetBool.Request(data=False))
            self.cli_boundary_follower_.call_async(SetBool.Request(data=True))
        elif self.state_ == 3:
            self.cli_motion_to_goal_.call_async(SetBool.Request(data=False))
            self.cli_boundary_follower_.call_async(SetBool.Request(data=False))

    def set_robot_position(self):
        entity_state = SetEntityState.Request()
        entity_state.state.name = "burger"
        entity_state.state.pose.position.x = self.initial_position_.x
        entity_state.state.pose.position.y = self.initial_position_.y
        future = self.cli_set_entity_state_.call_async(entity_state)
        rclpy.spin_until_future_complete(self, future)

    def calc_dist_points(self, point1, point2):
        dist = math.sqrt((point1.y - point2.y) ** 2 + (point1.x - point2.x) ** 2)
        return dist

    def normalize_angle(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def is_goal_reachable(self):
        desired_yaw = math.atan2(
            self.desired_position_.y - self.position_.y,
            self.desired_position_.x - self.position_.x,
        )
        err_yaw = self.normalize_angle(desired_yaw - self.yaw_)
        if err_yaw < 0:
            err_yaw = 2 * math.pi + err_yaw
        laser_index = int(err_yaw / math.pi * 180)
        if self.laser_data_[laser_index] < 1.0 and not math.isinf(
            self.laser_data_[laser_index]
        ):
            return False
        return True

    def update(self):
        if self.regions_ is None:
            return

        if self.state_ == 0:
            # MOVE TOWARDS GOAL UNTIL BOUNDARY IS FOUND
            if 0.15 < self.regions_["front"] < 1.25:
                self.get_logger().warning("Got to starting point in wall")
                self.circumnavigate_closest_point_ = self.position_
                self.circumnavigate_starting_point_ = self.position_
                self.change_state(1)

        elif self.state_ == 1:
            # FOLLOWS BOUNDARY UNTIL A COMPLETE LAP IS MADE
            # if current position is closer to the goal than the previous closest_position, assign current position to closest_point
            if self.calc_dist_points(
                self.position_, self.desired_position_
            ) < self.calc_dist_points(
                self.circumnavigate_closest_point_, self.desired_position_
            ):
                self.circumnavigate_closest_point_ = self.position_

            # compare only after 10 seconds - need some time to get out of starting_point
            # if robot reaches (is close to) starting point
            if (
                self.count_state_time_ > 10
                and self.calc_dist_points(
                    self.position_, self.circumnavigate_starting_point_
                )
                < 0.6
            ):
                self.get_logger().warning("moving to closest point")
                self.change_state(2)

        elif self.state_ == 2:
            # MOVES TOWARD GOAL ONCE CLOSEST POINT IS FOUND
            # if robot reaches (is close to) closest point
            if (
                self.calc_dist_points(
                    self.position_, self.circumnavigate_closest_point_
                )
                < 0.6
            ):
                if not self.is_goal_reachable():
                    self.get_logger().error("The goal is not reachable.")
                    self.change_state(3)
                    self.destroy_node()
                self.get_logger().warning(
                    "Got to closest point, now moving in direction to goal"
                )
                self.change_state(0)

        self.count_loop_ += 1
        if self.count_loop_ == 20:
            self.count_state_time_ += 1
            self.count_loop_ = 0


def main(args=None):
    rclpy.init(args=args)
    bug_node = Bug1()
    rclpy.spin(bug_node)

    bug_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
