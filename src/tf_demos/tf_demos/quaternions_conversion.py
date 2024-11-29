import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class QuaternionConversions(Node):
    def __init__(self):
        super().__init__("quaternion_conversions")

        # ! QUATERNION TO EULER CONVERSION
        # Create a list of floats, which is compatible with tf
        # quaternion methods
        quat_tf = [0.0, 1.0, 0.0, 0.0]

        quat_msg = Quaternion()
        quat_msg.x = quat_tf[0]
        quat_msg.y = quat_tf[1]
        quat_msg.z = quat_tf[2]
        quat_msg.w = quat_tf[3]
        self.get_logger().warn("Quaternion msg:")
        print(quat_msg)
        self.get_logger().info("################\n")

        # euler from quaternion

        (roll, pitch, yaw) = euler_from_quaternion(
            [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        )

        self.get_logger().warn("Quaternion to euler")
        self.get_logger().info("roll: %s >> pitch: %s >> yaw: %s" % (roll, pitch, yaw))
        self.get_logger().info("###########\n")

        # ! EULER TO QUATERNION CONVERSION
        self.get_logger().warn("Euler to quaternion:")
        quat_list = quaternion_from_euler(roll, pitch, yaw)
        self.get_logger().info(
            "The quaternion representation is x: %s >> y: %s >> %s z: %s."
            % (quat_list[0], quat_list[1], quat_list[2], quat_list[3])
        )

        # ================================


def main(args=None):
    rclpy.init(args=args)

    quaternion_conversions = QuaternionConversions()

    quaternion_conversions.destroy_node()
    rclpy.shutdown()
