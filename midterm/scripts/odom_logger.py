#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract orientation (quaternion → yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(f'Position -> x: {x:.2f}, y: {y:.2f}, Yaw: {math.degrees(yaw):.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
