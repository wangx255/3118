#!/usr/bin/env python3

# import ros stuff
import rclpy
from rclpy.node import Node

# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations
from gazebo_msgs.srv import SetEntityState

# import ros service
from std_srvs.srv import SetBool

import math


class Bug2(Node):
    def __init__(self):
        super().__init__("bug2_node")

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
        self.count_state_time_ = 0  # seconds the robot is in a state
        self.count_loop_ = 0
        self.laser_data_ = []
        self.past_mlines_ = []

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
        self.count_state_time_ = 0
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

    def set_robot_position(self):
        entity_state = SetEntityState.Request()
        entity_state.state.name = "burger"
        entity_state.state.pose.position.x = self.initial_position_.x
        entity_state.state.pose.position.y = self.initial_position_.y
        self.cli_set_entity_state_.call_async(entity_state)

    def distance_to_line(self, p0):
        # p0 is the current position
        # p1 and p2 points define the line
        p1 = self.initial_position_
        p2 = self.desired_position_
        # here goes the equation
        up_eq = math.fabs(
            (p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x)
        )
        lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
        distance = up_eq / lo_eq

        return distance

    def normalize_angle(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def is_new_mline(self):
        for past_mline in self.past_mlines_:
            dist_difference = math.sqrt(
                math.pow(self.position_.x - past_mline.x, 2)
                + math.pow(self.position_.y - past_mline.y, 2)
            )
            if dist_difference <= 0.1:
                return False
        return True

    def is_goal_reachable(self):
        desired_yaw = math.atan2(
            self.desired_position_.y - self.position_.y,
            self.desired_position_.x - self.position_.x,
        )
        err_yaw = self.normalize_angle(desired_yaw - self.yaw_)
        if err_yaw < 0:
            err_yaw = 2 * math.pi + err_yaw
        laser_index = int(err_yaw / math.pi * 180)
        if self.laser_data_[laser_index] < 1.25 and not math.isinf(
            self.laser_data_[laser_index]
        ):
            return False
        return True

    def main_callback(self):
        if self.regions_ is None:
            return

        distance_position_to_line = self.distance_to_line(self.position_)

        if self.state_ == 0:
            # MOVING TOWARDS THE GOAL UNTIL BOUNDARY IS FOUND
            if 0.15 < self.regions_["front"] < 1:
                self.change_state(1)

        elif self.state_ == 1:
            # FOLLOWS BOUNDARY UNTIL THE M-LINE TO THE GOAL
            if (
                self.count_state_time_ > 10
                and distance_position_to_line < 0.1
                and self.is_new_mline()
            ):
                self.past_mlines_.append(self.position_)
                self.get_logger().warn("Found M-Line, moving towards goal.")
                if not self.is_goal_reachable():
                    self.get_logger().error("The goal is not reachable.")
                    self.change_state(2)
                    self.destroy_node()
                self.change_state(0)

        self.count_loop_ += 1
        if self.count_loop_ == 20:
            self.count_state_time_ += 1
            self.count_loop_ = 0


def main(args=None):
    rclpy.init(args=args)
    bug2_node = Bug2()
    rclpy.spin(bug2_node)

    bug2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
