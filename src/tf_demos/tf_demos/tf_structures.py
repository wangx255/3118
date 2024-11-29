import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point, Pose


class TFStructures(Node):
    def __init__(self):
        super().__init__("quaternion_conversions")

        # ! DEFINE VECTOR3
        vector_msg = Vector3()
        vector_msg.x = 1.0
        vector_msg.y = 1.0
        vector_msg.z = 1.0
        self.get_logger().warn("Vector3 msg:")
        print(vector_msg)
        self.get_logger().info("################\n")

        # ================================

        # ! DEFINE POINT
        point_msg = Point()
        point_msg.x = 1.0
        point_msg.y = 1.0
        point_msg.z = 1.0
        self.get_logger().warn("Point msg:")
        print(point_msg)
        self.get_logger().info("################\n")

        # ================================

        # ! DEFINE POSE
        pose_msg = Pose()
        pose_msg.position.x = 1.0
        pose_msg.position.y = 1.0
        pose_msg.position.z = 1.0
        pose_msg.orientation.x = 1.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 0.0

        self.get_logger().warn("Pose msg:")
        print(pose_msg)
        self.get_logger().info("################\n")

        # ================================


def main(args=None):
    rclpy.init(args=args)

    tf_structures = TFStructures()

    tf_structures.destroy_node()
    rclpy.shutdown()
