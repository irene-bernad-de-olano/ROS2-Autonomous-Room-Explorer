#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Twist


class MarkerFollower(Node):

    def __init__(self):
        super().__init__('follower')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter('marker_frame', 'marker_18')
        self.declare_parameter('robot_frame', 'base_link')

        # Desired distance from the marker [meters]
        self.declare_parameter('desired_distance', 0.50)

        # Controller gains
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.5)

        # Maximum velocities
        self.declare_parameter('max_linear_speed', 0.20)
        self.declare_parameter('max_angular_speed', 0.80)

        # Stop when the marker is this close to desired distance
        self.declare_parameter('distance_tolerance', 0.05)

        # ---------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------

        self.marker_frame = self.get_parameter(
            'marker_frame'
        ).value

        self.robot_frame = self.get_parameter(
            'robot_frame'
        ).value

        self.desired_distance = self.get_parameter(
            'desired_distance'
        ).value

        self.linear_gain = self.get_parameter(
            'linear_gain'
        ).value

        self.angular_gain = self.get_parameter(
            'angular_gain'
        ).value

        self.max_linear_speed = self.get_parameter(
            'max_linear_speed'
        ).value

        self.max_angular_speed = self.get_parameter(
            'max_angular_speed'
        ).value

        self.distance_tolerance = self.get_parameter(
            'distance_tolerance'
        ).value

        # ---------------------------------------------------------
        # TF2
        # ---------------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(seconds=10.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------
        # Velocity publisher
        # ---------------------------------------------------------

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/create3/cmd_vel',
            10
        )


        # ---------------------------------------------------------
        # Timer
        # ---------------------------------------------------------

        self.timer = self.create_timer(
            0.1,       # 10 Hz
            self.control_loop
        )

        self.get_logger().info(
            f'Marker follower started. '
            f'Following {self.marker_frame} '
            f'from {self.robot_frame}.'
        )

    # -------------------------------------------------------------
    # Main control loop
    # -------------------------------------------------------------

    def control_loop(self):

        try:

            # Get transform:
            #
            # robot_frame -> marker_frame
            #
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.marker_frame,
                rclpy.time.Time()
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            self.get_logger().warn(
                f'Cannot find TF from '
                f'{self.robot_frame} to '
                f'{self.marker_frame}',
                throttle_duration_sec=2.0
            )

            # Stop if marker cannot be seen
            self.stop_robot()

            return

        # ---------------------------------------------------------
        # Extract marker position
        # ---------------------------------------------------------

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        self.get_logger().info(
            f'Marker: x={x:.2f} m, y={y:.2f} m',
            throttle_duration_sec=1.0
        )

        # ---------------------------------------------------------
        # Distance control
        # ---------------------------------------------------------

        error_distance = x - self.desired_distance

        # ---------------------------------------------------------
        # If marker is too close -> stop
        # ---------------------------------------------------------

        if abs(error_distance) < self.distance_tolerance:

            linear_velocity = 0.0

        else:

            linear_velocity = (
                self.linear_gain * error_distance
            )

        # ---------------------------------------------------------
        # Don't drive backwards.
        #
        # We only want the robot to approach the marker.
        # ---------------------------------------------------------

        if linear_velocity < 0.0:
            linear_velocity = 0.0

        # Limit linear velocity

        linear_velocity = min(
            linear_velocity,
            self.max_linear_speed
        )

        # ---------------------------------------------------------
        # Angular control
        #
        # If marker is on one side, turn toward it.
        # ---------------------------------------------------------

        angular_velocity = (
            self.angular_gain * y
        )

        # Limit angular velocity

        angular_velocity = max(
            -self.max_angular_speed,
            min(
                angular_velocity,
                self.max_angular_speed
            )
        )

        # ---------------------------------------------------------
        # Publish command
        # ---------------------------------------------------------

        cmd = Twist()

        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        self.cmd_vel_pub.publish(cmd)

    # -------------------------------------------------------------
    # Stop robot
    # -------------------------------------------------------------

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = MarkerFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()