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

        # ---------------------------------------------------------
        # IMPORTANT:
        # How long we allow the marker TF to disappear before
        # stopping the robot.
        #
        # 0.2 seconds = robot must have a recent marker detection.
        # ---------------------------------------------------------
        self.declare_parameter('marker_timeout', 0.20)

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

        self.marker_timeout = self.get_parameter(
            'marker_timeout'
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
        # CMD_VEL
        # =========================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/create3/cmd_vel',
            10
        )

        # =========================================================
        # STATE
        # =========================================================

        # Time when we last successfully saw the marker.
        self.last_marker_time = None

        # Remember whether we are currently stopped because
        # the marker disappeared. This prevents log spam.
        self.marker_missing = True

        # =========================================================
        # CONTROL LOOP
        # =========================================================

        self.timer = self.create_timer(
            0.1,
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
            f'Marker timeout: '
            f'{self.marker_timeout:.2f} s'
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
        # Get current time
        # ---------------------------------------------------------

        now = self.get_clock().now()

        # ---------------------------------------------------------
        # Get transform robot -> marker
        # ---------------------------------------------------------

        try:

            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.marker_frame,
                rclpy.time.Time()
            )

            # -----------------------------------------------------
            # We successfully received a marker transform.
            # -----------------------------------------------------

            self.last_marker_time = now

            if self.marker_missing:

                self.get_logger().info(
                    'Marker detected - FOLLOWING'
                )

            self.marker_missing = False

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            # -----------------------------------------------------
            # Marker TF is currently unavailable.
            #
            # IMPORTANT:
            #
            # We DO NOT continue using the previous command.
            #
            # We immediately stop the robot.
            # -----------------------------------------------------

            self.stop_robot()

            if not self.marker_missing:

                self.get_logger().warn(
                    'Marker lost - STOPPING ROBOT'
                )

            self.marker_missing = True

            return

        # =========================================================
        # ADDITIONAL TF TIMEOUT SAFETY
        # =========================================================
        #
        # Even if lookup_transform() returns something from the TF
        # buffer, make sure it is recent enough.
        #
        # This prevents the robot from following stale TF data.
        # =========================================================

        if self.last_marker_time is None:

            self.stop_robot()

            return

        marker_age = (
            now - self.last_marker_time
        ).nanoseconds / 1e9

        if marker_age > self.marker_timeout:

            self.stop_robot()

            if not self.marker_missing:

                self.get_logger().warn(
                    f'Marker data too old '
                    f'({marker_age:.2f} s) - STOPPING ROBOT'
                )

            self.marker_missing = True

            return

        # ---------------------------------------------------------
        # Marker position
        #
        # Robot forward direction = NEGATIVE X
        #
        # Therefore a marker in front normally has:
        #
        # x < 0
        #
        # ---------------------------------------------------------

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        # ---------------------------------------------------------
        # Distance
        # ---------------------------------------------------------

        distance = math.sqrt(
            x * x +
            y * y
        )

        # ---------------------------------------------------------
        # Angle relative to robot forward direction
        #
        # Forward = -X
        #
        # angle = atan2(y, -x)
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
        # STOP AT TARGET
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

        if abs(angle) < self.angle_deadband:

            angular_velocity = 0.0

        else:

            # Your robot requires the inverted direction.
            angular_velocity = (
                -self.angular_gain *
                angle
            )

        # Limit angular velocity

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

        distance_error = (
            distance -
            self.desired_distance
        )

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

        angle_abs = abs(angle)

        if angle_abs >= self.max_drive_angle:

            # Marker is too far to the side.
            #
            # STOP FORWARD MOTION.
            #
            # The robot rotates until the marker is closer
            # to the center.
            #

            linear_velocity = 0.0

        else:

            # Gradually reduce forward speed when not aligned.

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
        # PUBLISH COMMAND
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

            # This can happen if ROS has already shut down.
            pass

    #============================================================

def main(args=None):

    rclpy.init(args=args)

    node = MarkerFollower()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Stop the robot while ROS is still alive.

        if rclpy.ok():

            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()

if __name__ == '__main__':
    main()