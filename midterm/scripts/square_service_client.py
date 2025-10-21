#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class SquareServiceClient(Node):
    def __init__(self):
        super().__init__('square_service_client')
        self.client = self.create_client(Empty, 'square_service')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.req = Empty.Request()

    def send_request(self):
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    client = SquareServiceClient()
    response = client.send_request()
    client.get_logger().info('Service call completed.')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
