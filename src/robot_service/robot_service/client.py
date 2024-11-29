import sys

from robot_msgs.srv import AddTwoNumbers
import rclpy
from rclpy.node import Node


class AddTwoNumbersClient(Node):

    def __init__(self):
        super().__init__("add_two_numbers_client_node")
        self.cli = self.create_client(AddTwoNumbers, "add_two_numbers_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = AddTwoNumbers.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    add_two_numbers_client = AddTwoNumbersClient()
    response = add_two_numbers_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    add_two_numbers_client.get_logger().info(
        "Result of add_two_ints: for %d + %d = %d"
        % (int(sys.argv[1]), int(sys.argv[2]), response.sum)
    )

    add_two_numbers_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
