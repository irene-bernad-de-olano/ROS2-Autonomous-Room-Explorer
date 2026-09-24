#!/usr/bin/env python3

from math import atan2, hypot, isfinite

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time


class Follower(Node):
    def __init__(self, name='follower', approach_only=False):
        super().__init__(name)
        
        self.marker = self.declare_parameter('marker_frame', 'marker_18').value
        self.distance = self.declare_parameter('target_distance', 0.40).value
        
        if not isfinite(self.distance) or self.distance <= 0.0:
            raise ValueError('target_distance must be positive and finite')

        self.approach_only = approach_only
        self.reached = False
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.pub = self.create_publisher(
            Twist,
            'cmd_vel',
            1
        )
        self.timer = self.create_timer(0.1, self.control_callback)

    def marker_position(self):
        try:
            trans = self.buffer.lookup_transform('base_link', self.marker, Time())
        except tf2_ros.TransformException:
            return None

        age = (self.get_clock().now() - Time.from_msg(trans.header.stamp)).nanoseconds * 1e-9
        p = trans.transform.translation
        if not (
            0.0 <= age <= 0.5
            and isfinite(p.x)
            and isfinite(p.y)
            and p.x > 0.0):
            return None
        return p.x, p.y

    def stop(self):
        self.pub.publish(Twist())

    def control_callback(self):
        position = self.marker_position()
        if position is None or self.reached:
            self.stop()
            return

        x, y = position
        distance = hypot(x, y)
        bearing = atan2(y, x)
        error = distance - self.distance
        self.get_logger().info(f"Distance: {distance}")
        cmd = Twist()

        if (
            self.approach_only
            and distance <= self.distance
            and abs(bearing) < 0.10):
            self.reached = True
            self.stop()
            self.get_logger().info(f"Stopping because distance {distance}")
            return

        if abs(bearing) > 0.05:
            cmd.angular.z = max(-0.30, min(0.30, 1.0 * bearing))

        # Turn first when the marker is far off-centre.
        if abs(bearing) < 0.35 and abs(error) > 0.03:
            cmd.linear.x = max(-0.10, min(0.10, 0.4 * error))
            if self.approach_only:
                cmd.linear.x = max(0.0, cmd.linear.x)

        self.pub.publish(cmd)


def main(args=None):
    # Keep ROS alive during Ctrl+C cleanup so we can request a stop.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Follower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    finally:
        node.timer.cancel()
        if rclpy.ok():
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()