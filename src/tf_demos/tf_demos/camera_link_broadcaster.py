import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time


class CameraLinkBroadcaster(Node):
    def __init__(self):
        super().__init__("camera_link_broadcaster")

        # Initialize the transform broadcaster

        self.target_frame = (
            self.declare_parameter("target_frame", "base_link")
            .get_parameter_value()
            .string_value
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        wheel_base_link_tf = self.tf_buffer.lookup_transform(
            "wheel_left_link", self.target_frame, rclpy.time.Time()
        )

        camera_link_tf = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        camera_link_tf.header.stamp = self.time
        camera_link_tf.header.frame_id = "base_scan"
        camera_link_tf.child_frame_id = "camera_link"

        # defining traslation
        camera_link_tf.transform.translation.x = 0.0
        camera_link_tf.transform.translation.y = 0.0
        camera_link_tf.transform.translation.z = 0.1

        # defining orientation
        camera_link_tf.transform.rotation = wheel_base_link_tf.transform.rotation

        # Send the transformation
        self.tf_broadcaster.sendTransform(camera_link_tf)


def main(args=None):
    rclpy.init(args=args)

    camera_link_broadcaster = CameraLinkBroadcaster()

    rclpy.spin(camera_link_broadcaster)

    camera_link_broadcaster.destroy_node()
    rclpy.shutdown()
