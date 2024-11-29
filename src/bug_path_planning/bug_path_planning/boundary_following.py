# import ros stuff
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class BoundaryFollower(Node):
    def __init__(self):
        super().__init__("boundary_following_node")

        self.active_ = False

        self.regions_ = {
            "right": 0,
            "fright": 0,
            "front": 0,
            "fleft": 0,
            "left": 0,
        }
        self.state_ = 0

        self.state_dict_ = {
            0: "find a boundary",
            1: "turn right",
            2: "follow the boundary",
        }

        self.pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_ = self.create_subscription(LaserScan, "/scan", self.clbk_laser, 10)
        self.srv_ = self.create_service(
            SetBool, "boundary_follower_switch", self.boundary_follower_switch
        )

        self.create_timer(0.05, self.update)

    # CALLBACKS
    def clbk_laser(self, msg):
        self.regions_ = {
            "right": min(min(msg.ranges[269:270]), 10),
            "fright": min(min(msg.ranges[314:315]), 10),
            "front": min(min(msg.ranges[0:1]), 10),
            "fleft": min(min(msg.ranges[44:45]), 10),
            "left": min(min(msg.ranges[89:90]), 10),
        }
        self.take_action()

    # FUNCTIONS TO CONTROL THE SERVICE
    def boundary_follower_switch(self, req, response):
        self.active_ = req.data
        response.success = True
        response.message = "Done!"
        return response

    def change_state(self, state):
        if state != self.state_:
            self.get_logger().info(
                "boundary follower - [%s] - %s" % (state, self.state_dict_[state])
            )
            self.state_ = state

    # FUNCTION TO IDENTIFY WHICH ACTION SHOULD BE TAKEN ACCORDING TO THE LIDAR MEASUREMENTS
    def take_action(self):
        regions = self.regions_

        d = 1.25

        if regions["front"] > d and regions["fleft"] > d and regions["fright"] > d:
            self.change_state(0)
        elif regions["front"] < d and regions["fleft"] > d and regions["fright"] > d:
            self.change_state(1)
        elif (
            regions["front"] > d + 0.15
            and regions["fleft"] > d
            and regions["fright"] < d + 0.15
        ):
            self.change_state(2)
        elif regions["front"] > d and regions["fleft"] < d and regions["fright"] > d:
            self.change_state(0)
        elif regions["front"] < d and regions["fleft"] > d and regions["fright"] < d:
            self.change_state(1)
        elif regions["front"] < d and regions["fleft"] < d and regions["fright"] > d:
            self.change_state(1)
        elif regions["front"] < d and regions["fleft"] < d and regions["fright"] < d:
            self.change_state(1)
        elif regions["front"] > d and regions["fleft"] < d and regions["fright"] < d:
            self.change_state(0)
        else:
            self.get_logger().info(str(regions))

    # DIFFERENT ACTIONS OF THE BOUNDARY FOLLOWING
    def find_boundary(self):
        msg = Twist()
        self.get_logger().info("Finding boundary")
        msg.linear.x = 0.15
        msg.angular.z = -0.3
        return msg

    def turn_left(self):
        self.get_logger().info("Turning left")
        msg = Twist()
        msg.angular.z = 0.7
        return msg

    def follow_the_boundary(self):
        self.get_logger().info("Following the boundary")
        msg = Twist()
        msg.linear.x = 0.4
        return msg

    def update(self):
        if not self.active_:
            return

        msg = Twist()

        # ACTION TAKEN
        if self.state_ == 0:
            msg = self.find_boundary()
        elif self.state_ == 1:
            msg = self.turn_left()
        elif self.state_ == 2:
            msg = self.follow_the_boundary()
        else:
            self.get_logger().error("Unknown state!")

        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryFollower()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
