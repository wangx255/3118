import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3


class Controller(Node):
    def __init__(self):
        super().__init__("controller_py")

        self.pub_ = self.create_publisher(Vector3, "/act_setpoint", 10)

        self.sub_ = self.create_subscription(Point, "/odometry", self.odomCallback, 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.actPub)

        self.pos_x_ = 0.0
        self.pos_y_ = 0.0
        self.pos_z_ = 0.0
        self.k_ = 3.0  # controller constant, example

    def actPub(self):  # calculate control action
        force_msg = Vector3()

        force_msg.x = self.k_ * self.pos_x_
        force_msg.y = self.k_ * self.pos_y_
        force_msg.z = self.k_ * self.pos_z_
        # example
        self.pub_.publish(force_msg)

    def odomCallback(self, pos_msg):
        self.pos_x_ = pos_msg.x
        self.pos_y_ = pos_msg.y
        self.pos_z_ = pos_msg.z


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
