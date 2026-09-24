#!/usr/bin/env python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Twist


class MarkerFollower(Node):

    def __init__(self):
        super().__init__('follower')

        # =========================================================
        # PARAMETERS
        # =========================================================

        self.declare_parameter('marker_frame', 'marker_18')
        self.declare_parameter('robot_frame', 'base_link')

        # Desired distance from robot to marker
        self.declare_parameter('desired_distance', 0.01)

        # Stop tolerance
        self.declare_parameter('distance_tolerance', 0.025)

        # Linear control gain
        self.declare_parameter('linear_gain', 0.7)

        # Angular control gain
        self.declare_parameter('angular_gain', 0.8)

        # Maximum speeds
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.30)

        # Do not drive forward if marker is too far to the side
        self.declare_parameter('max_drive_angle', 0.70)

        # Ignore very small angular errors
        self.declare_parameter('angle_deadband', 0.05)

        # =========================================================
        # READ PARAMETERS
        # =========================================================

        self.marker_frame = self.get_parameter(
            'marker_frame'
        ).value

        self.robot_frame = self.get_parameter(
            'robot_frame'
        ).value

        self.desired_distance = self.get_parameter(
            'desired_distance'
        ).value

        self.distance_tolerance = self.get_parameter(
            'distance_tolerance'
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

        self.max_drive_angle = self.get_parameter(
            'max_drive_angle'
        ).value

        self.angle_deadband = self.get_parameter(
            'angle_deadband'
        ).value

        # =========================================================
        # TF2
        # =========================================================

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(seconds=10.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =========================================================
        # CREATE 3 CMD_VEL
        # =========================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/create3/cmd_vel',
            10
        )

        # =========================================================
        # CONTROL LOOP
        # =========================================================

        self.timer = self.create_timer(
            0.1,       # 10 Hz
            self.control_loop
        )

        # =========================================================
        # STARTUP
        # =========================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Marker follower started'
        )

        self.get_logger().info(
            f'Marker frame: {self.marker_frame}'
        )

        self.get_logger().info(
            f'Robot frame: {self.robot_frame}'
        )

        self.get_logger().info(
            f'Desired distance: '
            f'{self.desired_distance:.2f} m'
        )

        self.get_logger().info(
            'Robot forward direction = NEGATIVE X'
        )

        self.get_logger().info(
            'Angular steering direction = INVERTED'
        )

        self.get_logger().info(
            '========================================'
        )

    # =============================================================
    # CONTROL LOOP
    # =============================================================

    def control_loop(self):

        # ---------------------------------------------------------
        # Get transform robot -> marker
        # ---------------------------------------------------------

        try:

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
                'Marker TF not available - STOPPING ROBOT',
                throttle_duration_sec=2.0
            )

            self.stop_robot()

            return

        # ---------------------------------------------------------
        # Marker position
        #
        # In your coordinate system:
        #
        #       FRONT
        #         ^
        #         |
        #         |
        #        -X
        #
        # Therefore the marker is normally:
        #
        #       x < 0
        #
        # ---------------------------------------------------------

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        # ---------------------------------------------------------
        # Euclidean distance
        # ---------------------------------------------------------

        distance = math.sqrt(
            x * x +
            y * y
        )

        # ---------------------------------------------------------
        # Marker angle relative to robot forward direction
        #
        # Forward = -X
        #
        # Therefore:
        #
        # angle = atan2(y, -x)
        #
        # ---------------------------------------------------------

        angle = math.atan2(
            y,
            -x
        )

        angle_deg = math.degrees(angle)

        # ---------------------------------------------------------
        # DEBUG
        # ---------------------------------------------------------

        self.get_logger().info(
            f'Marker: '
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'd={distance:.3f} m, '
            f'angle={angle_deg:.1f} deg',
            throttle_duration_sec=0.5
        )

        # =========================================================
        # STOP CONDITION
        # =========================================================
        #
        # IMPORTANT:
        #
        # Stop whenever the robot is at or INSIDE the target
        # distance.
        #
        # This fixes the previous problem where:
        #
        # distance = 0.08 m
        # desired  = 0.20 m
        #
        # caused linear velocity to become zero but angular
        # velocity continued forever.
        #
        # We do NOT want:
        #
        # distance < desired_distance
        #
        # to cause rotation.
        #
        # =========================================================

        stop_distance = (
            self.desired_distance +
            self.distance_tolerance
        )

        if distance <= stop_distance:

            self.stop_robot()

            self.get_logger().info(
                f'Target reached: '
                f'{distance:.2f} m '
                f'(target {self.desired_distance:.2f} m) '
                f'- STOPPING',
                throttle_duration_sec=1.0
            )

            return

        # =========================================================
        # ANGULAR CONTROL
        # =========================================================

        # ---------------------------------------------------------
        # Small angle deadband
        #
        # If marker is almost straight ahead, don't rotate.
        # This prevents constant tiny left/right corrections.
        # ---------------------------------------------------------

        if abs(angle) < self.angle_deadband:

            angular_velocity = 0.0

        else:

            # IMPORTANT:
            #
            # The previous version turned in the wrong direction.
            #
            # Therefore the sign is intentionally NEGATIVE here.
            #
            angular_velocity = (
                -self.angular_gain *
                angle
            )

        # ---------------------------------------------------------
        # Limit angular velocity
        # ---------------------------------------------------------

        angular_velocity = max(
            -self.max_angular_speed,
            min(
                angular_velocity,
                self.max_angular_speed
            )
        )

        # =========================================================
        # LINEAR CONTROL
        # =========================================================

        # Distance still to travel

        distance_error = (
            distance -
            self.desired_distance
        )

        # Proportional controller

        linear_velocity = (
            self.linear_gain *
            distance_error
        )

        # Never drive backwards

        linear_velocity = max(
            0.0,
            linear_velocity
        )

        # =========================================================
        # STEERING / FORWARD SPEED
        # =========================================================
        #
        # If the marker is strongly to the side, don't drive
        # forward.
        #
        # First turn toward the marker.
        #
        # Once the marker is reasonably centered, drive forward.
        # =========================================================

        angle_abs = abs(angle)

        if angle_abs >= self.max_drive_angle:

            linear_velocity = 0.0

        else:

            # Reduce speed when marker is not centered.
            #
            # angle = 0
            #       -> full speed
            #
            # angle = max_drive_angle
            #       -> zero speed

            alignment_factor = (
                1.0 -
                angle_abs /
                self.max_drive_angle
            )

            alignment_factor = max(
                0.0,
                min(
                    1.0,
                    alignment_factor
                )
            )

            linear_velocity *= alignment_factor

        # =========================================================
        # LIMIT LINEAR SPEED
        # =========================================================

        linear_velocity = min(
            linear_velocity,
            self.max_linear_speed
        )

        # =========================================================
        # PUBLISH
        # =========================================================

        cmd = Twist()

        cmd.linear.x = linear_velocity
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_velocity

        self.cmd_vel_pub.publish(cmd)

    # =============================================================
    # STOP ROBOT
    # =============================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        try:
            self.cmd_vel_pub.publish(cmd)
        except Exception:
            pass


# =================================================================
# MAIN
# =================================================================

def main(args=None):

    rclpy.init(args=args)

    node = MarkerFollower()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Stop before shutting down ROS
        node.stop_robot()

        node.destroy_node()

        # Only shutdown if still running
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
