import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocitySub(Node):
    def __init__(self):
        super().__init__("velocity_sub_py")

        self.get_logger().info("Starting {0} node.".format(self.get_name()), once=True)

        self.vel_sub_ = self.create_subscription(
            Twist, "/turtle1/cmd_vel", self.velocity_cb, 10
        )

    def velocity_cb(self, velocity_msg: Twist):

        self.get_logger().info(
            "Turtle's velocity > \n x_linear:{0} \n z_angular:{1}".format(
                velocity_msg.linear.x, velocity_msg.angular.z
            ),
            once=False,
        )  # receiving velocity message information and printing it


def main(args=None):
    rclpy.init(args=args)

    velocity_sub = VelocitySub()

    rclpy.spin(velocity_sub)

    velocity_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
