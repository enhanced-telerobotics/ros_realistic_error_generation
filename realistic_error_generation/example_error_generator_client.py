import sys
import rclpy
from rclpy.node import Node
from realistic_error_generation.utils import numpy2msg
from realistic_error_generation_interfaces.srv import ObtainError
import numpy as np


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__("error_generation_client")
        self.cli = self.create_client(ObtainError, "ObtainError")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.req = ObtainError.Request()

    def send_request(self):
        # example
        previous_jp = np.array([0.2, 0.2, 0.125, 0.2, 0.2, 0.2])
        current_jp = np.array([0.2, 0.2, 0.125, 0.2, 0.2, 0.2])
        self.req.current_js = numpy2msg(current_jp)
        self.req.previous_js = numpy2msg(previous_jp)

        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request()
    rclpy.spin_until_future_complete(minimal_client, future)
    response: ObtainError.Response = future.result()

    minimal_client.get_logger().info(f"Error noise: {response.joint_error}")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
