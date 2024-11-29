import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__("tf_broadcaster")

        # Initialize the transform broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_scan"
        t.child_frame_id = "camera_link"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        quat = quaternion_from_euler(float(0), float(0), float(0))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    tf_broadcaster = StaticTFBroadcaster()

    rclpy.spin(tf_broadcaster)

    tf_broadcaster.destroy_node()
    rclpy.shutdown()
