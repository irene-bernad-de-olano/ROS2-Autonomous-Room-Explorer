import time


import rclpy

from rclpy.action import ActionClient

from rclpy.executors import ExternalShutdownException

from rclpy.node import Node

from std_msgs.msg import String

from whisper_msgs.action import STT


# whisper_ros's /whisper/listen action can hang forever (no result callback ever

# fires) when the captured audio transcribes to a completely empty string - seen

# in practice on a very short (~0.15s) VAD-triggered clip. This is the maximum

# time we wait for a result before cancelling the goal and asking again, so a

# single bad clip doesn't permanently kill voice control. A silent room legitimately

# leaves a goal outstanding for a long time too, so this must stay well above any

# realistic single-utterance turnaround.

RESULT_TIMEOUT_S = 20.0



class VoiceToCommand(Node):

    """Bridges whisper_ros's /whisper/listen action to our /user_command topic.


    whisper_ros (whisper.cpp based) does the actual speech-to-text; this node

    just repeatedly asks it for the next utterance and republishes the text on

    the same /user_command topic the turtlesim/Create 3 LLM controllers already

    subscribe to, so they need no changes at all.

    """


    def __init__(self):

        super().__init__('voice_to_command')

        self._pub = self.create_publisher(String, '/user_command', 10)

        self._retry_timer = None

        self._goal_handle = None

        self._goal_sent_at = None


        self._client = ActionClient(self, STT, '/whisper/listen')

        self.get_logger().info('Waiting for the Whisper action server (/whisper/listen)...')

        self._client.wait_for_server()

        self.get_logger().info('Whisper is ready, waiting for speech...')

        self.create_timer(2.0, self._check_watchdog)

        self.listen()


    def listen(self):

        self._goal_handle = None

        self._goal_sent_at = time.monotonic()

        self._client.send_goal_async(STT.Goal()).add_done_callback(self._on_goal)


    def _on_goal(self, future):

        handle = future.result()

        if not handle.accepted:

            self.get_logger().warn('Whisper rejected the goal, retrying in 1.0s')

            self._retry_timer = self.create_timer(1.0, self._retry)

            return

        self._goal_handle = handle

        handle.get_result_async().add_done_callback(self._on_result)


    def _retry(self):

        self.destroy_timer(self._retry_timer)

        self._retry_timer = None

        self.listen()


    def _on_result(self, future):

        self._goal_handle = None

        self._goal_sent_at = None

        text = future.result().result.transcription.text

        self.get_logger().info(f"Heard: '{text}'")

        if text.strip():

            self._pub.publish(String(data=text))

        self.listen()


    def _check_watchdog(self):

        if self._goal_sent_at is None or self._goal_handle is None:

            return

        elapsed = time.monotonic() - self._goal_sent_at

        if elapsed > RESULT_TIMEOUT_S:

            self.get_logger().warn(

                f"No response from Whisper for {elapsed:.1f}s (known hang on an empty "

                "transcript) - cancelling the goal and starting again."

            )

            self._goal_handle.cancel_goal_async()

            self.listen()



def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = VoiceToCommand()

        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):

        pass

    finally:

        if node is not None:

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()



if __name__ == '__main__':

    main()