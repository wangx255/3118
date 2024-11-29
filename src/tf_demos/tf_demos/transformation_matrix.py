import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformListener
from math import cos, sin, radians
import numpy as np
import time
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time


def trig(angle):
    r = radians(angle)
    return cos(r), sin(r)


def matrix(rotation, translation):
    xC, xS = trig(rotation[0])
    yC, yS = trig(rotation[1])
    zC, zS = trig(rotation[2])
    dX = translation[0]
    dY = translation[1]
    dZ = translation[2]

    Rotate_X_matrix = np.array([[1, 0, 0], [0, xC, -xS], [0, xS, xC]])
    Rotate_Y_matrix = np.array([[yC, 0, yS], [0, 1, 0], [-yS, 0, yC]])
    Rotate_Z_matrix = np.array([[zC, -zS, 0], [zS, zC, 0], [0, 0, 1]])

    rotation_matrix = Rotate_X_matrix * Rotate_Y_matrix * Rotate_Z_matrix

    full_matrix = np.eye(4)

    full_matrix[:3, :3] = rotation_matrix
    full_matrix[:3, 3] = np.array([dX, dY, dZ])
    return full_matrix


class TFMatrixExample(Node):
    def __init__(self):
        super().__init__("tf_matrix_example")

        # Initialize the transform broadcaster and listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self.target_frame = (
            self.declare_parameter("target_frame", "base_link")
            .get_parameter_value()
            .string_value
        )

        self.tf_buffer.wait_for_transform_async(
            "can_link", "base_link", rclpy.time.Time()
        )
        self.tf_buffer.wait_for_transform_async(
            "camera_link", "base_link", rclpy.time.Time()
        )

        self.timer = self.create_timer(0.1, self.broadcast_callback)

        # ===============================================

    def listener_callback(self, msg):
        self.time.sec = msg.clock.sec
        self.time.nanosec = msg.clock.nanosec

    def broadcast_callback(self):

        # ! MANUAL TRANSFORMATION MATRIX
        self.get_logger().warn("CALCULATING HOMOGENEUS TRANSFORMATION MATRIX MANUALLY")

        # Matrixes from base_link to camera_link
        t1 = (0, 0, 0.27)
        r1 = (0, 0, 0)

        camera_base_matrix = matrix(r1, t1)

        self.get_logger().info(
            "Homogeneus transformation matrix from BASE_LINK to CAMERA_LINK (DEFINED MANUALLY):"
        )
        self.get_logger().info("T0 = ")
        print(np.round(camera_base_matrix, decimals=2))
        self.get_logger().info("###############")

        time.sleep(2)

        # Matrix from camera_link to can_link
        t1 = (0.3, 0, -0.28)
        r1 = (0, 0, 0)

        can_camera_matrix = matrix(r1, t1)

        self.get_logger().info(
            "Homogeneus transformation matrix from CAMERA_LINK to CAN_LINK (DEFINED MANUALLY):"
        )
        self.get_logger().info("T1 = ")
        print(np.round(can_camera_matrix, decimals=2))
        self.get_logger().info("#################")

        time.sleep(2)

        result_matrix = np.dot(camera_base_matrix, can_camera_matrix)
        self.get_logger().info(
            "Resultant transformation matrix from BASE_LINK TO CAN_LINK through CAMERA_LINK (T0*T1):"
        )
        self.get_logger().info("T0*T1 = ")
        print(np.round(result_matrix, decimals=2))

        time.sleep(3)

        self.get_logger().info("==================")

        # ! GETTING MATRIX WITH TF LIBRARY

        self.get_logger().warn(
            "OBTAINING HOMOGENEUS TRANSFORMATION MATRIX AUTOMATICALLY"
        )

        # USING TF LIBRARIES

        self.tf_listener = TransformListener(self.tf_buffer, self)

        rclpy.spin_once(self)

        can_link_tf = self.tf_buffer.lookup_transform(
            "base_link",
            "can_link",
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=10.0),
        )

        self.get_logger().info(
            "Homogeneus transformation matrix from BASE_LINK frame to CAN_LINK frame by using ROS TF libraries:"
        )
        print(can_link_tf)

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    tf_matrix_example = TFMatrixExample()

    rclpy.spin(tf_matrix_example)

    tf_matrix_example.destroy_node()
    rclpy.shutdown()
