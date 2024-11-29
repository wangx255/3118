# import ros stuff
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_srvs.srv import SetBool
import math


class MotionToGoal(Node):
    def __init__(self):
        super().__init__("motion_to_goal")

        self.active_ = False

        # robot state variables
        self.position_ = Point()
        self.yaw_ = 0
        # machine state
        self.state_ = 0

        # goal
        self.desired_position_ = Point()
        # In ROS2, parameters are accessed differently
        self.desired_position_.x = 1.0  # default value, can be overridden by parameters
        self.desired_position_.y = 1.0  # default value, can be overridden by parameters
        self.desired_position_.z = 0.0
        # parameters
        self.yaw_precision_ = math.pi / 90  # +/- 2 degree allowed
        self.dist_precision_ = 0.3

        self.pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_odom_ = self.create_subscription(Odometry, "/odom", self.clbk_odom, 10)
        self.srv_ = self.create_service(
            SetBool, "motion_to_goal_switch", self.motion_to_goal_switch
        )

        # Parameters
        self.declare_parameter("des_pos_x", 1.0)
        self.declare_parameter("des_pos_y", 1.0)
        self.desired_position_.x = (
            self.get_parameter("des_pos_x").get_parameter_value().double_value
        )
        self.desired_position_.y = (
            self.get_parameter("des_pos_y").get_parameter_value().double_value
        )

        self.create_timer(0.05, self.update)

    # CALLBACKS
    def clbk_odom(self, msg):
        # recording position of robot
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

    # FUNCTIONS TO CONTROL SERVICE
    def motion_to_goal_switch(self, req, response):
        self.active_ = req.data
        response.success = True
        response.message = "Done!"
        return response

    def change_state(self, state):
        self.state_ = state
        self.get_logger().info(f"State changed to [{self.state_}]")

    # DIFFERENT ACTIONS FOR MOTION TO GOAL
    #  =======================================================
    def fix_yaw(self, des_pos):
        desired_yaw = math.atan2(
            des_pos.y - self.position_.y, des_pos.x - self.position_.x
        )
        err_yaw = self.normalize_angle(desired_yaw - self.yaw_)

        twist_msg = Twist()
        if math.fabs(math.fabs(err_yaw)) > self.yaw_precision_:
            twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7

        self.pub_.publish(twist_msg)

        # state change conditions
        if math.fabs(math.fabs(err_yaw)) <= self.yaw_precision_:
            self.change_state(1)

    def go_straight_ahead(self, des_pos):
        desired_yaw = math.atan2(
            des_pos.y - self.position_.y, des_pos.x - self.position_.x
        )
        err_yaw = desired_yaw - self.yaw_
        err_pos = math.sqrt(
            pow(des_pos.y - self.position_.y, 2) + pow(des_pos.x - self.position_.x, 2)
        )

        if err_pos > self.dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
            self.pub_.publish(twist_msg)
        else:
            self.change_state(2)

        # state change conditions
        if math.fabs(err_yaw) > self.yaw_precision_:
            self.change_state(0)

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub_.publish(twist_msg)

    #  =======================================================

    def normalize_angle(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def update(self):
        if not self.active_:
            return

        # ACTION TAKEN
        if self.state_ == 0:
            self.get_logger().info("Orienting towards goal")
            self.fix_yaw(self.desired_position_)
        elif self.state_ == 1:
            self.get_logger().info("Moving towards goal")
            self.go_straight_ahead(self.desired_position_)
        elif self.state_ == 2:
            self.get_logger().warning("Got to goal")
            self.done()
            self.destroy_node()
        else:
            self.get_logger().error("Unknown state!")


def main(args=None):
    rclpy.init(args=args)
    node = MotionToGoal()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
