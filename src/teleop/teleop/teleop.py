import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class JoyToCmdVel(Node):

    def __init__(self):
        super().__init__('joy_to_cmd_vel')

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Subscriber for joystick input
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Speed scaling
        self.linear_scale = 0.26
        self.angular_scale = 0.6

        # Button configuration
        # Change these numbers depending on your controller
        self.deadman_button = 4
        self.emergency_button = 5

        # Emergency stop state
        self.emergency_stop = False

        # Emergency stop service
        self.emergency_service = self.create_service(
            Trigger,
            'emergency_stop',
            self.emergency_stop_callback
        )

        self.get_logger().info('Joystick teleop started')
        self.get_logger().info(
            'Dead Man button: ' + str(self.deadman_button)
        )
        self.get_logger().info(
            'Emergency Stop button: ' + str(self.emergency_button)
        )

    def joy_callback(self, msg):

        # Create a zero velocity command
        twist = Twist()

        # Check that the required buttons exist
        if len(msg.buttons) <= max(
            self.deadman_button,
            self.emergency_button
        ):
            self.get_logger().warn(
                'Joystick does not have enough buttons.'
            )
            self.publisher_.publish(twist)
            return

        # Emergency stop button
        if msg.buttons[self.emergency_button] == 1:

            self.emergency_stop = True

            # Immediately stop the robot
            self.publisher_.publish(twist)

            self.get_logger().warn(
                'EMERGENCY STOP ACTIVATED'
            )
            return

        # If emergency stop has been activated,
        # ignore joystick commands
        if self.emergency_stop:

            self.publisher_.publish(twist)
            return

        # Dead Man's Switch
        # Robot only moves while this button is held
        if msg.buttons[self.deadman_button] != 1:

            self.publisher_.publish(twist)
            return

        # Joystick control is allowed
        if len(msg.axes) >= 2:

            twist.linear.x = (
                float(msg.axes[1]) *
                self.linear_scale
            )

            twist.angular.z = (
                float(msg.axes[0]) *
                self.angular_scale
            )

        self.publisher_.publish(twist)

    def emergency_stop_callback(self, request, response):

        # Reset the emergency stop
        self.emergency_stop = False

        response.success = True
        response.message = 'Emergency stop reset'

        self.get_logger().info(
            'Emergency stop reset. Joystick control enabled.'
        )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = JoyToCmdVel()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Make sure the robot stops when the node exits
        stop_cmd = Twist()
        node.publisher_.publish(stop_cmd)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
