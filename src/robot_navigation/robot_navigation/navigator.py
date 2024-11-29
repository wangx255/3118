import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator_py")
        self.pub_ = self.create_publisher(Point, "/odometry", 10)
        self.timer_ = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # calculate position
        pos_msg = Point()
        pos_msg.x = 1.0
        pos_msg.y = 2.0
        pos_msg.z = 5.0

        self.pub_.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()

    rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    navigator = Navigator()
    navigator.start()
