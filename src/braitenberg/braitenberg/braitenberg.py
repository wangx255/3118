import rclpy
from rclpy.node import Node
import tf2_ros
import math
from std_msgs.msg import Float64
import sys
import time
from std_msgs.msg import Float64MultiArray


class Braitenberg(Node):
    def __init__(self):
        super().__init__("braitenberg_node")
        self.models_pos_ = dict()

        # TRANSFORM LISTENER
        # USED TO LISTEN TO THE DISTANCES BETWEEN THE SENSORS
        # AND THE OBJECT OF INTEREST
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # BRAITENBERG VEHICLE MODE
        self.mode = "fear"
        if len(sys.argv) > 1:
            self.mode = str(sys.argv[1])

        # SENSORS FIELD OF VIEW
        self.sensors_fov = math.pi / 2
        self.max_speed = 10

        # =================================
        #! PUBLISHERS
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )
        # ==================================

        self.timer = self.create_timer(0.1, self.send_velocities)

    def define_wheels_vel(
        self, mode, l_sensor_dis, r_sensor_dis, l_sensor_ang, r_sensor_ang
    ):
        """Defines the velocity of the wheels depending on the distance and angle of the simulated sensors to the object of interest"""

        l_wheel_vel = 0
        r_wheel_vel = 0

        # FEAR MODE
        # =========================
        if mode == "fear":
            left_sphere_in_fov = 1.6 > l_sensor_ang > -0.05
            right_sphere_in_fov = -1.6 < r_sensor_ang < 0.17
            if left_sphere_in_fov:
                l_wheel_vel = self.max_speed
                r_wheel_vel = -self.max_speed
            elif right_sphere_in_fov:
                l_wheel_vel = -self.max_speed
                r_wheel_vel = self.max_speed
            else:
                l_wheel_vel = self.max_speed / (l_sensor_dis * 0.5)
                r_wheel_vel = self.max_speed / (r_sensor_dis * 0.5)
        # =========================

        # AGGRESSION MODE
        # =========================
        elif mode == "aggression":
            left_sphere_in_fov = 3.14 > l_sensor_ang > 0.4
            right_sphere_in_fov = -3.14 < r_sensor_ang < -0.4
            if left_sphere_in_fov:
                r_wheel_vel = self.max_speed
                l_wheel_vel = -self.max_speed
            elif right_sphere_in_fov:
                l_wheel_vel = self.max_speed
                r_wheel_vel = -self.max_speed
            else:
                l_wheel_vel = self.max_speed
                r_wheel_vel = self.max_speed
        # =========================

        # LOVE MODE
        # =========================
        elif mode == "love":
            left_sphere_in_fov = 3.14 > l_sensor_ang > 0.4
            right_sphere_in_fov = -3.14 < r_sensor_ang < -0.4
            if left_sphere_in_fov:
                r_wheel_vel = self.max_speed
                l_wheel_vel = -self.max_speed
            elif right_sphere_in_fov:
                l_wheel_vel = self.max_speed
                r_wheel_vel = -self.max_speed
            else:
                l_wheel_vel = (
                    self.max_speed
                    / ((l_sensor_dis - 1.5) * 0.1)
                    * math.cos(l_sensor_ang)
                )
                r_wheel_vel = (
                    self.max_speed
                    / ((r_sensor_dis - 1.5) * 0.1)
                    * math.cos(r_sensor_ang)
                )
        # =========================

        # EXPLORATION MODE
        # =========================
        elif mode == "exploration":
            left_sphere_in_fov = 1.6 > l_sensor_ang > -0.05
            right_sphere_in_fov = -1.6 < r_sensor_ang < 0.17
            if left_sphere_in_fov:
                l_wheel_vel = self.max_speed
                r_wheel_vel = -self.max_speed
            elif right_sphere_in_fov:
                l_wheel_vel = -self.max_speed
                r_wheel_vel = self.max_speed
            else:
                l_wheel_vel = self.max_speed
                r_wheel_vel = self.max_speed
        # =========================

        l_wheel_vel = min(max(l_wheel_vel, -self.max_speed), self.max_speed)
        r_wheel_vel = min(max(r_wheel_vel, -self.max_speed), self.max_speed)

        return l_wheel_vel, r_wheel_vel

    def send_velocities(self):
        """Main function for the execution of the code"""

        # LISTENING TO THE TRANSFORM FROM SENSORS TO OBJECT OF INTEREST
        # =============================================================
        try:
            lsensor_trans = self.tf_buffer.lookup_transform(
                "left_light_sensor", "unit_sphere", rclpy.time.Time()
            )
            rsensor_trans = self.tf_buffer.lookup_transform(
                "right_light_sensor", "unit_sphere", rclpy.time.Time()
            )

            self.lsensor_trans = [
                lsensor_trans.transform.translation.x,
                lsensor_trans.transform.translation.y,
            ]
            self.rsensor_trans = [
                rsensor_trans.transform.translation.x,
                rsensor_trans.transform.translation.y,
            ]
            # =============================================================

            # CALCULATING SENSOR DISTANCE
            # =============================================================
            lsensor_distance = math.sqrt(
                self.lsensor_trans[0] ** 2 + self.lsensor_trans[1] ** 2
            )
            rsensor_distance = math.sqrt(
                self.rsensor_trans[0] ** 2 + self.rsensor_trans[1] ** 2
            )
            # =============================================================

            # CALCULATING SENSOR ANGLE
            # =============================================================
            lsensor_angle = math.atan2(self.lsensor_trans[1], self.lsensor_trans[0])
            rsensor_angle = math.atan2(self.rsensor_trans[1], self.rsensor_trans[0])
            # =============================================================

            # DEFINING WHEELS VELOCITY ACCORDING TO VEHICLE MODE, DISTANCE TO SENSORS AND ANGLE
            # =============================================================
            l_wheel_vel, r_wheel_vel = self.define_wheels_vel(
                self.mode,
                lsensor_distance,
                rsensor_distance,
                lsensor_angle,
                rsensor_angle,
            )
            # =============================================================

            l_wheel_vel = float(l_wheel_vel)
            r_wheel_vel = float(r_wheel_vel)

            self.get_logger().info("###############")
            self.get_logger().info(
                f"Wheels velocities: left: {l_wheel_vel}, right: {r_wheel_vel}"
            )
            self.get_logger().info("###############")

            # PUBLISHING VELOCITIES DEPENDING ON THE MODE
            # =============================================================
            data = [l_wheel_vel, r_wheel_vel]

            # =============================================================
            msg = Float64MultiArray()
            msg.data = data
            self.wheel_pub.publish(msg)

        except tf2_ros.LookupException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    braitenberg_node = Braitenberg()
    rclpy.spin(braitenberg_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
