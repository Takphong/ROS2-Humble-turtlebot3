#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math
import time

class SquareServiceServer(Node):
    def __init__(self):
        super().__init__('square_service_server')
        self.srv = self.create_service(Empty, 'square_service', self.execute_callback)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Square Service Server ready.')

    def move_straight(self, speed, distance):
        twist = Twist()
        twist.linear.x = speed
        duration = distance / speed
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        twist.linear.x = 0.0
        self.publisher.publish(twist)

    def turn(self, angular_speed, angle):
        twist = Twist()
        twist.angular.z = angular_speed
        duration = angle / angular_speed
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def execute_callback(self, request, response):
        self.get_logger().info('Service called: Moving in a square.')
        speed = 0.1          # m/s
        side_length = 0.5    # meters
        turn_speed = 0.5     # rad/s
        turn_angle = math.pi / 1.5 # 90 degrees

        self.move_straight(speed, side_length)
        time.sleep(1)
        self.turn(turn_speed, 1.3)
        time.sleep(1)

        for i in range(3):
            self.move_straight(speed, side_length)
            time.sleep(1)
            self.turn(turn_speed, turn_angle)
            time.sleep(1)
            
        # Stop at the end
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        self.get_logger().info('Finished square path.')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SquareServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
