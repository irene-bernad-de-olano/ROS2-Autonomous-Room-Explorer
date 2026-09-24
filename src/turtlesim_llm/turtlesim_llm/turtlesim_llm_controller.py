import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from std_msgs.msg import String

import requests

import json

import time

import threading

import math


class TurtlesimLLMController(Node):

    def __init__(self):

        super().__init__('turtlesim_llm_controller')


        self.declare_parameter('ollama_url', 'http://192.168.88.111:11434/api/generate')

        self.declare_parameter('model', 'qwen3:4b')


        self.ollama_url = self.get_parameter('ollama_url').get_parameter_value().string_value

        self.model = self.get_parameter('model').get_parameter_value().string_value


        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.command_sub = self.create_subscription(String, '/user_command', self.command_callback, 10)


        # IMPORTANT: A lock, so that only one command is ever processed at a time!

        self.is_busy = threading.Lock()

###     - If 'circle': Provide 'direction' ('left' or 'right') and 'value' (circle radius in meters, default 1.5 if unspecified).

        self.system_prompt = """

        You are a command extractor for a robot. Extract the user's intent into a sequence of actions.

        Output ONLY a JSON object with an 'actions' array.


        Action rules:

        - 'type': Must be either 'move', 'turn' or 'circle'.

        - If 'move': Provide 'value' (distance in meters).

        - If 'turn': Provide 'direction' ('left' or 'right') and 'value' (in degrees).

        - If 'circle': Provide 'direction' ('left' or 'right'), 'value' (circle radius in meters, default 1.0 if unspecified), and 'fraction' (fraction of a full circle, default 1.0).

        

        Example 1: "Turn left by 30 degrees then move 2m forward"

        {"actions": [{"type": "turn", "direction": "left", "value": 30}, {"type": "move", "value": 2.0}]}


        Example 2: "Move backward 1 meter, then turn right 90 degrees"

        {"actions": [{"type": "move", "value": -1.0}, {"type": "turn", "direction": "right", "value": 90}]}


        Example 3: "Draw a circle" or "Drive in a loop to the right"

        {"actions": [{"type": "circle", "direction": "left", "value": 1.5}]}

        

        Example 4: "Drive a half circle to the left"

        {"actions": [{"type": "circle", "direction": "left", "value": 1.0, "fraction": 0.5}]}

        Example 5: "Drive 4/7 of a circle to the left"

        {"actions": [{"type": "circle", "direction": "left", "value": 1.0, "fraction": 0.5714}]}

        """


        self.get_logger().info(f"Robust LLM Controller started. Model: {self.model}")


    def command_callback(self, msg):

        user_text = msg.data


        # Check whether the robot/controller is still busy

        if self.is_busy.locked():

            self.get_logger().warn(f"Ignoring command ('{user_text}'), the robot is still busy!")

            return


        self.get_logger().info(f"Command received: '{user_text}'.")

        # Start the worker in a separate thread

        threading.Thread(target=self.process_and_execute, args=(user_text,)).start()


    def process_and_execute(self, user_text):

        # Lock so that no other commands interfere

        with self.is_busy:

            payload = {

                "model": self.model,

                "prompt": f"{self.system_prompt}\nUser Command: {user_text}",

                "stream": False,

                "format": "json",

                # Some models (e.g. qwen3) are "thinking" models and would otherwise

                # put their answer in "thinking" instead of "response".

                "think": False

            }


            try:

                # API request with a generous timeout

                response = requests.post(self.ollama_url, json=payload, timeout=60.0)

                response.raise_for_status()

                data = response.json()


                command_data = json.loads(data.get("response", "{}"))

                actions = command_data.get("actions", [])


                if not actions:

                    self.get_logger().info("No actions recognized.")

                    return


                self.get_logger().info(f"Starting execution of {len(actions)} actions...")


                for idx, action in enumerate(actions):

                    twist_msg = Twist()

                    act_type = action.get("type")

                    wait_time = 1.2  # move/turn: velocity == desired amount, held for ~1s


                    if act_type == "turn":

                        deg = float(action.get("value", 0.0))

                        rad = math.radians(deg)

                        direction = action.get("direction", "left")


                        if direction == "right":

                            twist_msg.angular.z = -rad

                        else:

                            twist_msg.angular.z = rad


                    elif act_type == "move":

                        twist_msg.linear.x = float(action.get("value", 0.0))


                    elif act_type == "circle":


                        # Circle arc with configurable fraction of a full circle
                        radius = float(action.get("value", 1.5)) or 1.5
                        fraction = float(action.get("fraction", 1.0))

                        speed = 1.0
                        angular_speed = speed / radius

                        direction = action.get("direction", "left")

                        if direction == "right":
                            angular_speed = -angular_speed

                        twist_msg.linear.x = speed
                        twist_msg.angular.z = angular_speed

                        # fraction = 1.0 -> 360°
                        # fraction = 0.5 -> 180° (half circle)
                        # fraction = 4/7 -> about 206°
                        wait_time = abs((2 * math.pi * fraction) / angular_speed)




                    #    # Circle = constant linear AND angular velocity at the same time,

                    #    # just like a real differential-drive robot.

                    #    radius = float(action.get("value", 1.5)) or 1.5

                    #    speed = 1.0

                    #    angular_speed = speed / radius

                    #    direction = action.get("direction", "left")


                    #    if direction == "right":

                    #        angular_speed = -angular_speed


                    #    twist_msg.linear.x = speed

                    #    twist_msg.angular.z = angular_speed

                    #    # A full revolution takes 2*pi / |angular_speed| seconds

                    #    wait_time = abs(2 * math.pi / angular_speed)


                    self.get_logger().info(f"Step {idx+1}: {act_type} -> linear: {twist_msg.linear.x}, angular: {twist_msg.angular.z}")


                    # Turtlesim automatically stops the turtle if no new cmd_vel message

                    # arrives for about 1s. For move/turn a single publish is enough

                    # (the action is designed for ~1s anyway), but 'circle' can take much

                    # longer, so keep republishing regularly for its whole duration so the

                    # watchdog never triggers.

                    republish_interval = 0.5

                    elapsed = 0.0

                    while elapsed < wait_time:

                        self.cmd_vel_pub.publish(twist_msg)

                        step = min(republish_interval, wait_time - elapsed)

                        time.sleep(step)

                        elapsed += step


                # Explicitly stop the robot at the end of the sequence (important after 'circle')

                self.cmd_vel_pub.publish(Twist())

                self.get_logger().info("Sequence fully completed. Ready for the next command.")


            except Exception as e:

                self.get_logger().error(f"Error during execution: {e}")


def main(args=None):

    rclpy.init(args=args)

    node = TurtlesimLLMController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()