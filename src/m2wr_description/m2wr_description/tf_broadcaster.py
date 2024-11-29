import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage


class ChassisTfBroadcaster(Node):
    def __init__(self):
        super().__init__("chassis_tf_broadcaster")
        self.br = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        qos_profile = QoSProfile(depth=10)
        self.models_pose_sub = self.create_subscription(
            ModelStates, "/gazebo/model_states", self.model_pose_cb, qos_profile
        )

        self.tf_sub = self.create_subscription(
            TFMessage, "/tf", self.tf_cb, qos_profile
        )

        self.chassis_position = [0.0, 0.0, 0.0]
        self.chassis_orientation = [0.0, 0.0, 0.0, 0.0]

        self.sphere_position = [0.0, 0.0, 0.0]
        self.sphere_orientation = [0.0, 0.0, 0.0, 0.0]

        self.timer = self.create_timer(0.01, self.send_transforms)

        self.stamp = 0

    def tf_cb(self, msg: TFMessage):
        self.stamp = msg.transforms[0].header.stamp

    def model_pose_cb(self, msg: ModelStates):
        models_list = msg.name
        models_pose = msg.pose

        try:
            chassis_index = models_list.index("m2wr")

            m2wr_position = models_pose[chassis_index].position
            m2wr_orientation = models_pose[chassis_index].orientation

            self.chassis_position = [m2wr_position.x, m2wr_position.y, m2wr_position.z]
            self.chassis_orientation = [
                m2wr_orientation.x,
                m2wr_orientation.y,
                m2wr_orientation.z,
                m2wr_orientation.w,
            ]
        except ValueError:
            pass

        try:
            sphere_index = models_list.index("unit_sphere")

            sphere_position = models_pose[sphere_index].position
            sphere_orientation = models_pose[sphere_index].orientation

            self.sphere_position = [
                sphere_position.x,
                sphere_position.y,
                sphere_position.z,
            ]
            self.sphere_orientation = [
                sphere_orientation.x,
                sphere_orientation.y,
                sphere_orientation.z,
                sphere_orientation.w,
            ]
        except ValueError:
            pass

    def send_transforms(self):
        if self.stamp != 0:
            self.send_transform(
                self.chassis_position, self.chassis_orientation, "link_chassis"
            )
            self.send_transform(
                self.sphere_position, self.sphere_orientation, "unit_sphere"
            )

    def send_transform(self, position, orientation, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.stamp
        t.header.frame_id = "world"
        t.child_frame_id = child_frame_id
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = ChassisTfBroadcaster()
    rclpy.spin(tf_broadcaster)

    tf_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
