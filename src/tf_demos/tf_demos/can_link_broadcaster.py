import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time


class CanTFBroadcaster(Node):
    def __init__(self):
        super().__init__("can_tf_broadcaster")

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.broadcast_callback)

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Clock, "/clock", self.listener_callback, qos_profile=qos_policy
        )

        self.clock = Clock()
        self.time = Time()

    def listener_callback(self, msg):
        self.time.sec = msg.clock.sec
        self.time.nanosec = msg.clock.nanosec

    def broadcast_callback(self):

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.time
        t.header.frame_id = "base_link"
        t.child_frame_id = "can_link"

        # defining traslation
        t.transform.translation.x = 0.3
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # defining orientation
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    can_tf_broadcaster = CanTFBroadcaster()

    rclpy.spin(can_tf_broadcaster)

    can_tf_broadcaster.destroy_node()
    rclpy.shutdown()
