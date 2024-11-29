from robot_msgs.srv import AddTwoNumbers
import rclpy
from rclpy.node import Node


class AddTwoNumbersService(Node):
    def __init__(self):
        super().__init__("add_two_numbers_service")
        self.srv = self.create_service(
            AddTwoNumbers, "add_two_numbers_service", self.add_two_numbers_callback
        )

    def add_two_numbers_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info("Incoming request\na: %d b: %d" % (request.a, request.b))

        return response


def main():
    rclpy.init()

    add_two_numbers_service = AddTwoNumbersService()

    add_two_numbers_service.get_logger().info("AddTwoNumbersService is running.")

    rclpy.spin(add_two_numbers_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
