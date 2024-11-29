import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage


class GroundTruthBroadcaster(Node):
    def __init__(self):
        super().__init__("groundtruth_broadcaster")

        self.declare_parameter("init_pose_x", 0)
        self.declare_parameter("init_pose_y", 0)

        self.init_pose_x = self.get_parameter("init_pose_x").value
        self.init_pose_y = self.get_parameter("init_pose_y").value

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

            self.chassis_position = [
                m2wr_position.x - self.init_pose_x,
                m2wr_position.y - self.init_pose_y,
                0.0,
            ]
            self.chassis_orientation = [
                m2wr_orientation.x,
                m2wr_orientation.y,
                m2wr_orientation.z,
                m2wr_orientation.w,
            ]
        except ValueError:
            pass

    def send_transforms(self):
        if self.stamp != 0:
            self.send_transform(
                self.chassis_position, self.chassis_orientation, "groundtruth_odom"
            )

    def send_transform(self, position, orientation, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.stamp
        t.header.frame_id = "odom"
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
    tf_broadcaster = GroundTruthBroadcaster()
    rclpy.spin(tf_broadcaster)

    tf_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
