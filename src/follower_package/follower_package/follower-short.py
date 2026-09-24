#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Twist


class MarkerFollower(Node):

    def __init__(self):
        super().__init__('follower')

        # ---------------------------------------------------------
        # Einstellungen
        # ---------------------------------------------------------

        self.marker = 'marker_18'
        self.robot = 'base_link'

        # Roboter soll bei 20 cm stoppen
        self.stop_distance = 0.20

        # Wie genau muss der Roboter zum Marker ausgerichtet sein?
        # 5 Grad
        self.angle_tolerance = math.radians(5.0)

        # Geschwindigkeiten
        self.forward_speed = 0.10
        self.turn_speed = 0.25

        # ---------------------------------------------------------
        # TF
        # ---------------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------
        # Create 3
        # ---------------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            '/create3/cmd_vel',
            10
        )

        # 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.control
        )

        self.get_logger().info(
            'Marker follower gestartet'
        )

    # =============================================================
    # CONTROL
    # =============================================================

    def control(self):

        # ---------------------------------------------------------
        # Marker suchen
        # ---------------------------------------------------------

        try:

            tf = self.tf_buffer.lookup_transform(
                self.robot,
                self.marker,
                rclpy.time.Time()
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            # Marker nicht sichtbar
            self.stop()
            return

        # ---------------------------------------------------------
        # Position des Markers
        # ---------------------------------------------------------

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        # Abstand zum Marker
        distance = math.sqrt(x*x + y*y)

        # ---------------------------------------------------------
        # Abstand erreicht?
        # ---------------------------------------------------------

        if distance <= self.stop_distance:

            self.stop()
            return

        # ---------------------------------------------------------
        # WICHTIG:
        #
        # Bei eurem TF ist NEGATIVES X = vorne.
        #
        # Deshalb berechnen wir den Winkel relativ zu -X.
        # ---------------------------------------------------------

        angle = math.atan2(y, -x)

        # ---------------------------------------------------------
        # ZUERST AUSRICHTEN
        # ---------------------------------------------------------

        if abs(angle) > self.angle_tolerance:

            cmd = Twist()

            # Marker links  -> positive Drehung
            # Marker rechts -> negative Drehung

            if angle > 0:
                cmd.angular.z = self.turn_speed
            else:
                cmd.angular.z = -self.turn_speed

            cmd.linear.x = 0.0

            self.cmd_pub.publish(cmd)
            return

        # ---------------------------------------------------------
        # Marker ist gerade vor dem Roboter
        # -> nach vorne fahren
        # ---------------------------------------------------------

        cmd = Twist()

        cmd.linear.x = self.forward_speed
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # =============================================================
    # STOP
    # =============================================================

    def stop(self):

        self.cmd_pub.publish(Twist())


# ================================================================
# MAIN
# ================================================================

def main():

    rclpy.init()

    node = MarkerFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            node.stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()