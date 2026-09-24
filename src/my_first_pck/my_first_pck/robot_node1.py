#!/usr/bin/env python3

import rclpy

def main():
    rclpy.init()
    node=rclpy.create_node('firstnode')
    rclpy.spin(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
