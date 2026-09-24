import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from std_msgs.msg import String

import requests

import json

import time

import threading

import math


class Create3LLMController(Node):

    def __init__(self):

        super().__init__('create3_llm_controller')


        self.declare_parameter('ollama_url', 'http://<OLLAMA_SERVER_IP>:11434/api/generate')

        self.declare_parameter('model', 'qwen3:4b')

        self.declare_parameter('linear_speed', 0.15)

        self.declare_parameter('angular_speed', 0.4)


        self.ollama_url = self.get_parameter('ollama_url').get_parameter_value().string_value

        self.model = self.get_parameter('model').get_parameter_value().string_value

        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value


        # The republisher from the Communication chapter provides cmd_vel without a namespace

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.command_sub = self.create_subscription(String, '/user_command', self.command_callback, 10)


        # IMPORTANT: A lock, so that only one command is ever processed at a time!

        self.is_busy = threading.Lock()


        self.system_prompt = """

        You are a command extractor for a robot. Extract the user's intent into a sequence of actions.

        Output ONLY a JSON object with an 'actions' array.


        Action rules:

        - 'type': Must be either 'move' or 'turn'.

        - If 'move': Provide 'value' (distance in meters).

        - If 'turn': Provide 'direction' ('left' or 'right') and 'value' (in degrees).


        Example 1: "Turn left by 30 degrees then move 2m forward"

        {"actions": [{"type": "turn", "direction": "left", "value": 30}, {"type": "move", "value": 2.0}]}


        Example 2: "Move backward 1 meter, then turn right 90 degrees"

        {"actions": [{"type": "move", "value": -1.0}, {"type": "turn", "direction": "right", "value": 90}]}

        """


        self.get_logger().info(f"Create 3 LLM Controller started. Model: {self.model}")


    def command_callback(self, msg):

        user_text = msg.data


        # Check whether the robot/controller is still busy

        if self.is_busy.locked():

            self.get_logger().warn(f"Ignoring command ('{user_text}'), the robot is still busy!")

            return


        self.get_logger().info(f"Command received: '{user_text}'.")

        # Start the worker in a separate thread

        threading.Thread(target=self.process_and_execute, args=(user_text,)).start()


    def stop_robot(self):

        self.cmd_vel_pub.publish(Twist())


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

                    duration = 0.0


                    if act_type == "turn":

                        deg = float(action.get("value", 0.0))

                        rad = math.radians(abs(deg))

                        direction = action.get("direction", "left")


                        twist_msg.angular.z = self.angular_speed if direction == "left" else -self.angular_speed

                        duration = rad / self.angular_speed


                    elif act_type == "move":

                        distance = float(action.get("value", 0.0))


                        twist_msg.linear.x = math.copysign(self.linear_speed, distance)

                        duration = abs(distance) / self.linear_speed


                    self.get_logger().info(

                        f"Step {idx+1}: {act_type} -> linear: {twist_msg.linear.x:.2f} m/s, "

                        f"angular: {twist_msg.angular.z:.2f} rad/s, duration: {duration:.2f}s"

                    )


                    # Drive at a fixed speed, republishing every 0.5s for the whole

                    # duration - a single "fire and forget" publish would leave a

                    # gap the Create 3's own cmd_vel watchdog treats as lost control

                    # and stops the robot on, cutting the motion short.

                    republish_interval = 0.5

                    elapsed = 0.0

                    while elapsed < duration:

                        self.cmd_vel_pub.publish(twist_msg)

                        step = min(republish_interval, duration - elapsed)

                        time.sleep(step)

                        elapsed += step

                    self.stop_robot()

                    time.sleep(0.3)  # short pause so the robot settles


                self.get_logger().info("Sequence fully completed. Robot stopped.")


            except Exception as e:

                self.get_logger().error(f"Error during execution: {e}")

                self.stop_robot()


def main(args=None):

    rclpy.init(args=args)

    node = Create3LLMController()

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