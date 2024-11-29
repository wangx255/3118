import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class VelocityPub(Node):
    def __init__(self):
        super().__init__("velocity_pub_py")
        self.get_logger().info("Starting {0} node.".format(self.get_name()), once=True)

        self.vel_msg_ = Twist()

        self.vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def start(self):
        while rclpy.ok():
            #! publishing velocities
            # move forward
            self.vel_msg_.linear.x = 1.0
            self.vel_msg_.angular.z = 0.0
            self.vel_pub_.publish(self.vel_msg_)
            time.sleep(2)

            # rotate
            self.vel_msg_.linear.x = 0.0
            self.vel_msg_.angular.z = 0.5
            self.vel_pub_.publish(self.vel_msg_)
            time.sleep(2)


def main(args=None):
    rclpy.init(args=args)

    velocity_pub = VelocityPub()
    velocity_pub.start()

    velocity_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
