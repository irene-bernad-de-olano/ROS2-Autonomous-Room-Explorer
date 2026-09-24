#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class HandSignController(Node):

    def __init__(self):
        super().__init__('hand_sign_controller')

        # Publisher for robot velocity
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber for YOLO hand signs
        self.sign_sub = self.create_subscription(
            String,
            '/hand_sign',
            self.sign_callback,
            10
        )

        # Speeds
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        # Current command
        self.current_command = "STOP"

        # Publish velocity continuously
        self.timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_cmd_vel
        )

        self.get_logger().info(
            'Hand sign controller started'
        )

    def sign_callback(self, msg):

        sign = msg.data

        if sign == "LEFT":

            self.current_command = "LEFT"
            self.get_logger().info("Command: LEFT")

        elif sign == "RIGHT":

            self.current_command = "RIGHT"
            self.get_logger().info("Command: RIGHT")

        elif sign == "UP":

            self.current_command = "UP"
            self.get_logger().info("Command: UP")

        elif sign == "STOP":

            self.current_command = "STOP"
            self.get_logger().info("Command: STOP")

    def publish_cmd_vel(self):

        twist = Twist()

        if self.current_command == "LEFT":

            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed

        elif self.current_command == "RIGHT":

            twist.linear.x = 0.0
            twist.angular.z = -self.angular_speed

        elif self.current_command == "UP":

            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        elif self.current_command == "STOP":

            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


def main():

    rclpy.init()

    node = HandSignController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()