#!/usr/bin/env python3

# ================================================================
# CONTENT / INHALTSVERZEICHNIS
# ================================================================
#
# 1. PARAMETERS / PARAMETER                    -> lines 20-38
# 2. TF2 SETUP / TF2-EINRICHTUNG               -> lines 40-45
# 3. CMD_VEL / ROBOTER-STEUERUNG               -> lines 47-53
# 4. MARKER POSITION / MARKERPOSITION           -> lines 58-75
# 5. STOP CONDITION / STOP-BEDINGUNG            -> lines 78-85
# 6. TURNING / DREHEN                           -> lines 88-101
# 7. FORWARD MOVEMENT / VORWÄRTSFAHREN          -> lines 104-121
# 8. PUBLISH COMMAND / BEFEHL SENDEN           -> lines 124-135
# 9. STOP ROBOT / ROBOTER STOPPEN               -> lines 138-145
#
# IMPORTANT SETTINGS / WICHTIGE EINSTELLUNGEN:
#
# marker_frame      = Marker-ID ändern
# desired_distance = gewünschter Abstand zum Marker
# linear_gain      = Beschleunigung / wie stark Abstand beeinflusst
# max_linear_speed = maximale Fahrgeschwindigkeit
# max_angular_speed= maximale Drehgeschwindigkeit
# max_drive_angle  = ab welchem Winkel zuerst gedreht wird
#
# Marker size / Markergröße wird NICHT hier eingestellt!
# Die Markergröße steht in aruco_opencv_params.yaml.
# Bei eurem aktuellen Setup: 0.144 m = 14.4 cm
# ================================================================


import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import Twist


class MarkerFollower(Node):

    def __init__(self):
        super().__init__('follower')

        # =========================================================
        # 1. PARAMETERS / PARAMETER
        # =========================================================

        # Marker to follow / Zu verfolgender Marker
        # CHANGE THIS if your marker ID is different!
        # ÄNDERN, wenn eure Marker-ID anders ist.
        self.marker_frame = self.declare_parameter(
            'marker_frame', 'marker_18'
        ).value

        # Desired distance in meters
        # Gewünschter Abstand in Metern
        # CHANGE THIS to set the stopping distance!
        # HIER ÄNDERN, um den gewünschten Abstand einzustellen.
        self.desired_distance = self.declare_parameter(
            'desired_distance', 0.20
        ).value

        # Distance tolerance / Toleranz des Abstands
        # CHANGE THIS if the robot stops too early/late.
        # HIER ÄNDERN, wenn der Roboter zu früh/spät stoppt.
        self.distance_tolerance = self.declare_parameter(
            'distance_tolerance', 0.03
        ).value

        # How strongly distance controls speed
        # Wie stark der Abstand die Geschwindigkeit beeinflusst
        # Higher = faster approach / Höher = schnelleres Zufahren
        self.linear_gain = self.declare_parameter(
            'linear_gain', 0.7
        ).value

        # Maximum forward speed in m/s
        # Maximale Vorwärtsgeschwindigkeit in m/s
        self.max_linear_speed = self.declare_parameter(
            'max_linear_speed', 0.15
        ).value

        # Maximum rotation speed in rad/s
        # Maximale Drehgeschwindigkeit in rad/s
        self.max_angular_speed = self.declare_parameter(
            'max_angular_speed', 0.30
        ).value

        # Maximum angle for driving forward
        # Maximaler Winkel, bei dem noch vorwärts gefahren wird
        #
        # Smaller = turn more before driving
        # Kleiner = Roboter dreht sich stärker zuerst aus
        #
        # 0.35 rad ≈ 20 degrees
        self.max_drive_angle = self.declare_parameter(
            'max_drive_angle', 0.35
        ).value

        # Small angle that is ignored
        # Kleine Winkelfehler werden ignoriert
        self.angle_deadband = self.declare_parameter(
            'angle_deadband', 0.05
        ).value

        # =========================================================
        # 2. TF2 SETUP / TF2-EINRICHTUNG
        # =========================================================

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =========================================================
        # 3. CMD_VEL / ROBOTER-STEUERUNG
        # =========================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/create3/cmd_vel',
            10
        )

        # Control loop: every 0.1 seconds = 10 Hz
        # Regelung: alle 0,1 Sekunden = 10 Hz
        #
        # This means the robot continuously recalculates
        # its direction and speed when the marker moves.
        # Dadurch wird bei einer Bewegung des Markers
        # ständig neu berechnet.
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

    # =============================================================
    # 4. MARKER POSITION / MARKERPOSITION
    # =============================================================

    def get_marker_position(self):

        try:
            # Get current marker position relative to robot
            # Aktuelle Markerposition relativ zum Roboter holen
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                self.marker_frame,
                Time()
            )

        except tf2_ros.TransformException:
            # Marker not visible / TF unavailable
            # Marker nicht sichtbar / TF nicht verfügbar
            return None

        p = transform.transform.translation

        # Ignore invalid values
        # Ungültige Werte ignorieren
        if not (
            math.isfinite(p.x)
            and math.isfinite(p.y)
        ):
            return None

        return p.x, p.y

    # =============================================================
    # 5. STOP CONDITION / STOP-BEDINGUNG
    # =============================================================

    def control_loop(self):

        position = self.get_marker_position()

        # ---------------------------------------------------------
        # Marker lost -> STOP
        # Marker verloren -> STOPP
        # ---------------------------------------------------------
        #
        # This is one of the important parts from Geraldo's code.
        # Das ist einer der wichtigen Teile aus Geraldos Code.
        #
        # We explicitly publish a zero Twist.
        # Wir senden ausdrücklich einen Twist mit Geschwindigkeit 0.
        if position is None:
            self.stop_robot()
            return

        x, y = position

        # =========================================================
        # Distance / Abstand
        # =========================================================

        distance = math.hypot(x, y)

        # =========================================================
        # Angle / Winkel
        # =========================================================
        #
        # IMPORTANT:
        # Your robot's forward direction is NEGATIVE X.
        # Die Vorwärtsrichtung eures Roboters ist NEGATIVES X.
        #
        angle = math.atan2(y, -x)

        # ---------------------------------------------------------
        # Reached target distance -> STOP
        # Zielabstand erreicht -> STOPP
        # ---------------------------------------------------------

        stop_distance = (
            self.desired_distance
            + self.distance_tolerance
        )

        if distance <= stop_distance:
            self.stop_robot()
            return

        # =========================================================
        # 6. TURNING / DREHEN
        # =========================================================

        # Small angle -> don't rotate
        # Kleiner Winkel -> nicht drehen
        if abs(angle) < self.angle_deadband:
            angular_velocity = 0.0

        else:
            # Proportional turning
            # Proportionales Drehen
            angular_velocity = -0.8 * angle

            # Limit rotation speed
            # Drehgeschwindigkeit begrenzen
            angular_velocity = max(
                -self.max_angular_speed,
                min(
                    angular_velocity,
                    self.max_angular_speed
                )
            )

        # =========================================================
        # 7. FORWARD MOVEMENT / VORWÄRTSFAHREN
        # =========================================================

        # Distance still to travel
        # Noch zurückzulegende Entfernung
        distance_error = (
            distance - self.desired_distance
        )

        # Proportional speed control
        # Proportionale Geschwindigkeitsregelung
        #
        # Example:
        # distance_error = 0.5 m
        # linear_gain    = 0.7
        #
        # speed = 0.35 m/s
        #
        # Afterwards the maximum speed is applied.
        # Danach wird die maximale Geschwindigkeit angewendet.
        linear_velocity = (
            self.linear_gain * distance_error
        )

        # Never drive backwards
        # Niemals rückwärts fahren
        linear_velocity = max(
            0.0,
            linear_velocity
        )

        # ---------------------------------------------------------
        # Turn first if marker is far to the side
        # Erst drehen, wenn Marker stark seitlich liegt
        # ---------------------------------------------------------

        if abs(angle) >= self.max_drive_angle:

            linear_velocity = 0.0

        else:

            # Slow down while marker is not centered
            # Langsamer fahren, solange Marker nicht mittig ist
            alignment_factor = (
                1.0
                - abs(angle) / self.max_drive_angle
            )

            alignment_factor = max(
                0.0,
                min(1.0, alignment_factor)
            )

            linear_velocity *= alignment_factor

        # Maximum forward speed
        # Maximale Vorwärtsgeschwindigkeit
        linear_velocity = min(
            linear_velocity,
            self.max_linear_speed
        )

        # =========================================================
        # 8. PUBLISH COMMAND / BEFEHL SENDEN
        # =========================================================

        cmd = Twist()

        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        self.cmd_vel_pub.publish(cmd)

    # =============================================================
    # 9. STOP ROBOT / ROBOTER STOPPEN
    # =============================================================

    def stop_robot(self):

        # Empty Twist = all velocities are zero
        # Leerer Twist = alle Geschwindigkeiten sind 0
        self.cmd_vel_pub.publish(Twist())


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
        # Always stop the robot before shutting down.
        # Roboter vor dem Beenden immer stoppen.
        node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    