#!/usr/bin/env python3

import os
import json
import math

import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, Spin

from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from .room_classifier import classify_rooms, get_evidence


class RoomExplorerNode(Node):

    def __init__(self):

        super().__init__('room_explorer_node')

        # ---------------------------------------------------------
        # PARAMETERS
        # ---------------------------------------------------------

        self.declare_parameter(
            'auto_start',
            False
        )

        self.declare_parameter(
            'min_confidence',
            0.5
        )

        self.declare_parameter(
            'rotation_angle',
            6.28318530718
        )

        self.declare_parameter(
            'rotation_time',
            30.0
        )

        self.declare_parameter(
            'max_goal_retries',
            1
        )

        # ---------------------------------------------------------
        # PARAMETERS
        # ---------------------------------------------------------

        self.auto_start = self.get_parameter(
            'auto_start'
        ).value

        self.min_confidence = self.get_parameter(
            'min_confidence'
        ).value

        self.rotation_angle = self.get_parameter(
            'rotation_angle'
        ).value

        self.rotation_time = self.get_parameter(
            'rotation_time'
        ).value

        self.max_goal_retries = self.get_parameter(
            'max_goal_retries'
        ).value

        # ---------------------------------------------------------
        # PUBLISHERS
        # ---------------------------------------------------------

        self.status_pub = self.create_publisher(
            String,
            '/room_explorer/status',
            10
        )

        self.report_pub = self.create_publisher(
            String,
            '/room_explorer/report',
            10
        )

        # ---------------------------------------------------------
        # SUBSCRIBERS
        # ---------------------------------------------------------

        self.detection_sub = self.create_subscription(
            String,
            '/yolo_detections_data',
            self.detection_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            '/user_command',
            self.command_callback,
            10
        )

        # ---------------------------------------------------------
        # ACTION CLIENTS
        # ---------------------------------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.spin_client = ActionClient(
            self,
            Spin,
            '/spin'
        )

        # ---------------------------------------------------------
        # LOAD ROOMS
        # ---------------------------------------------------------

        package_share = get_package_share_directory(
            'room_explorer'
        )

        rooms_file = os.path.join(
            package_share,
            'config',
            'rooms.yaml'
        )

        with open(rooms_file, 'r') as file:
            data = yaml.safe_load(file)

        self.rooms = data['rooms']

        # ---------------------------------------------------------
        # EXPLORATION STATE
        # ---------------------------------------------------------

        self.room_ids = list(self.rooms.keys())

        self.current_room_index = 0
        self.current_point_index = 0

        self.current_room = None
        self.current_point = None

        self.running = False
        self.collecting = False
        self.finishing = False

        self.goal_retries = 0

        self.requested_room = None

        # ---------------------------------------------------------
        # OBSERVATIONS
        # ---------------------------------------------------------

        self.observations = {
            room_id: []
            for room_id in self.room_ids
        }

        # ---------------------------------------------------------
        # START
        # ---------------------------------------------------------

        self.publish_status(
            "Room Explorer ready."
        )

        if self.auto_start:

            self.create_timer(
                2.0,
                self.start_auto_exploration,
                callback_group=None
            )

        self.get_logger().info(
            "Room Explorer started."
        )

    # =============================================================
    # STATUS
    # =============================================================

    def publish_status(self, text):

        msg = String()
        msg.data = text

        self.status_pub.publish(msg)

        self.get_logger().info(text)

    # =============================================================
    # VOICE COMMAND
    # =============================================================

    def command_callback(self, msg):

        command = msg.data.lower().strip()

        self.get_logger().info(
            f"Received command: {command}"
        )

        if self.running:
            self.publish_status(
                "Already exploring. Command ignored."
            )
            return

        requested = None

        if "bathroom" in command:
            requested = "Bathroom"

        elif "bedroom" in command:
            requested = "Bedroom"

        elif "kitchen" in command:
            requested = "Kitchen"

        elif "empty" in command:
            requested = "Empty"

        if requested is not None:

            self.requested_room = requested

            self.publish_status(
                f"Requested room type: {requested}"
            )

        else:

            self.requested_room = None

            self.publish_status(
                "No specific room requested. Exploring all rooms."
            )

        self.start_exploration()

    # =============================================================
    # AUTO START
    # =============================================================

    def start_auto_exploration(self):

        if self.running:
            return

        self.start_exploration()

    # =============================================================
    # START EXPLORATION
    # =============================================================

    def start_exploration(self):

        if self.running:
            return

        self.running = True
        self.finishing = False

        self.current_room_index = 0
        self.current_point_index = 0

        self.goal_retries = 0

        self.observations = {
            room_id: []
            for room_id in self.room_ids
        }

        self.publish_status(
            "Starting exploration of all four rooms."
        )

        if not self.nav_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.publish_status(
                "ERROR: /navigate_to_pose action server not available."
            )

            self.running = False
            return

        self.send_next_navigation_goal()

    # =============================================================
    # SEND NAVIGATION GOAL
    # =============================================================

    def send_next_navigation_goal(self):

        if self.current_room_index >= len(self.room_ids):

            self.finish_exploration()

            return

        room_id = self.room_ids[
            self.current_room_index
        ]

        points = self.rooms[room_id]['points']

        if self.current_point_index >= len(points):

            self.publish_status(
                f"{room_id}: all exploration points completed."
            )

            self.current_room_index += 1
            self.current_point_index = 0

            self.send_next_navigation_goal()

            return

        point = points[
            self.current_point_index
        ]

        self.current_room = room_id
        self.current_point = self.current_point_index + 1

        self.collecting = False
        self.goal_retries = 0

        self.publish_status(
            f"Navigating to {room_id}, "
            f"P{self.current_point}: "
            f"x={point['x']:.3f}, "
            f"y={point['y']:.3f}"
        )

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = float(
            point['x']
        )

        goal_msg.pose.pose.position.y = float(
            point['y']
        )

        # ---------------------------------------------------------
        # Optional yaw from YAML
        # ---------------------------------------------------------

        yaw = float(
            point.get('yaw', 0.0)
        )

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )

        goal_msg.pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        future = self.nav_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self.navigation_goal_response
        )

    # =============================================================
    # NAVIGATION GOAL RESPONSE
    # =============================================================

    def navigation_goal_response(self, future):

        goal_handle = future.result()

        if goal_handle is None:

            self.publish_status(
                "Navigation goal returned no goal handle."
            )

            self.skip_current_point()

            return

        if not goal_handle.accepted:

            self.publish_status(
                "Navigation goal rejected."
            )

            self.skip_current_point()

            return

        self.publish_status(
            "Navigation goal accepted."
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_result
        )

    # =============================================================
    # NAVIGATION RESULT
    # =============================================================

    def navigation_result(self, future):

        result = future.result()

        if result is None:

            self.publish_status(
                "Navigation returned no result."
            )

            self.skip_current_point()

            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:

            self.publish_status(
                f"Reached {self.current_room}, "
                f"P{self.current_point}."
            )

            self.begin_observation()

        else:

            self.publish_status(
                f"Navigation failed at "
                f"{self.current_room}, "
                f"P{self.current_point}. "
                f"Status={result.status}"
            )

            if self.goal_retries < self.max_goal_retries:

                self.goal_retries += 1

                self.publish_status(
                    f"Retrying navigation "
                    f"({self.goal_retries}/"
                    f"{self.max_goal_retries})."
                )

                self.send_next_navigation_goal()

            else:

                self.publish_status(
                    "Maximum retries reached. "
                    "Skipping point."
                )

                self.skip_current_point()

    # =============================================================
    # OBSERVATION / ROTATION
    # =============================================================

    def begin_observation(self):

        self.collecting = True

        self.publish_status(
            f"Observing {self.current_room}, "
            f"P{self.current_point}."
        )

        if not self.spin_client.wait_for_server(
            timeout_sec=1.0
        ):

            self.publish_status(
                "Spin action not available. "
                "Observing without rotation."
            )

            self.create_timer(
                3.0,
                self.finish_observation_once
            )

            return

        spin_goal = Spin.Goal()

        spin_goal.target_yaw = float(
            self.rotation_angle
        )

        spin_goal.time_allowance.sec = int(
            self.rotation_time
        )

        spin_goal.time_allowance.nanosec = 0

        self.publish_status(
            "Starting 360 degree observation."
        )

        future = self.spin_client.send_goal_async(
            spin_goal
        )

        future.add_done_callback(
            self.spin_goal_response
        )

    # =============================================================
    # SPIN RESPONSE
    # =============================================================

    def spin_goal_response(self, future):

        goal_handle = future.result()

        if goal_handle is None:

            self.publish_status(
                "Spin returned no goal handle."
            )

            self.finish_observation()

            return

        if not goal_handle.accepted:

            self.publish_status(
                "Spin goal rejected."
            )

            self.finish_observation()

            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.spin_result
        )

    # =============================================================
    # SPIN RESULT
    # =============================================================

    def spin_result(self, future):

        result = future.result()

        if result is None:

            self.publish_status(
                "Spin returned no result."
            )

        elif result.status == GoalStatus.STATUS_SUCCEEDED:

            self.publish_status(
                f"360 degree observation completed "
                f"for {self.current_room}, "
                f"P{self.current_point}."
            )

        else:

            self.publish_status(
                f"Spin finished with status "
                f"{result.status}."
            )

        self.finish_observation()

    # =============================================================
    # FINISH OBSERVATION
    # =============================================================

    def finish_observation_once(self):

        self.finish_observation()

    def finish_observation(self):

        if not self.collecting:
            return

        self.collecting = False

        count = len(
            self.observations[self.current_room]
        )

        self.publish_status(
            f"{self.current_room}, "
            f"P{self.current_point}: "
            f"{count} detections recorded."
        )

        self.current_point_index += 1

        self.send_next_navigation_goal()

    # =============================================================
    # DETECTION CALLBACK
    # =============================================================

    def detection_callback(self, msg):

        if not self.collecting:
            return

        try:

            detections = json.loads(
                msg.data
            )

        except json.JSONDecodeError:

            self.get_logger().warning(
                "Invalid YOLO JSON received."
            )

            return

        if not isinstance(detections, list):
            return

        for detection in detections:

            object_name = detection.get(
                'object',
                'unknown'
            )

            confidence = float(
                detection.get(
                    'confidence',
                    0.0
                )
            )

            observation = {
                "object": object_name,
                "confidence": confidence,
                "point": f"P{self.current_point}"
            }

            self.observations[
                self.current_room
            ].append(
                observation
            )

            self.get_logger().info(
                f"OBSERVATION | "
                f"{self.current_room} | "
                f"P{self.current_point} | "
                f"{object_name} | "
                f"confidence={confidence:.3f}"
            )

    # =============================================================
    # SKIP POINT
    # =============================================================

    def skip_current_point(self):

        self.collecting = False

        self.current_point_index += 1

        self.send_next_navigation_goal()

    # =============================================================
    # FINISH ALL ROOMS
    # =============================================================

    def finish_exploration(self):

        self.collecting = False
        self.finishing = True

        self.publish_status(
            "All four rooms explored. "
            "Starting final room classification."
        )

        try:

            assignments, scores = classify_rooms(
                self.observations,
                self.min_confidence
            )

        except Exception as error:

            self.publish_status(
                f"Classification error: {error}"
            )

            self.running = False
            return

        self.final_assignments = assignments
        self.final_scores = scores

        # ---------------------------------------------------------
        # Create final report
        # ---------------------------------------------------------

        report_lines = []

        report_lines.append(
            "FINAL ROOM CLASSIFICATION"
        )

        report_lines.append(
            "=========================="
        )

        for room_id in self.room_ids:

            room_type = assignments[room_id]

            evidence = scores[room_id]

            relevant = []

            for object_name in [
                "toilet",
                "bed",
                "potted plant",
                "oven",
                "microwave",
                "refrigerator"
            ]:

                for detection in self.observations[room_id]:

                    if (
                        detection["object"].lower()
                        == object_name
                        and detection["confidence"]
                        >= self.min_confidence
                    ):

                        relevant.append(
                            f"{object_name} "
                            f"({detection['confidence']:.2f})"
                        )

            if relevant:

                reason = ", ".join(
                    sorted(set(relevant))
                )

            else:

                reason = (
                    "no relevant object detected"
                )

            line = (
                f"{room_id}: "
                f"{room_type} "
                f"| {reason}"
            )

            report_lines.append(line)

        report = "\n".join(
            report_lines
        )

        # ---------------------------------------------------------
        # Publish
        # ---------------------------------------------------------

        msg = String()
        msg.data = report

        self.report_pub.publish(msg)

        self.publish_status(
            "Final classification completed."
        )

        for line in report_lines:

            self.get_logger().info(
                line
            )

        # ---------------------------------------------------------
        # Go to requested room
        # ---------------------------------------------------------

        if self.requested_room is not None:

            target_room_id = None

            for room_id, room_type in assignments.items():

                if room_type == self.requested_room:

                    target_room_id = room_id
                    break

            if target_room_id is not None:

                self.publish_status(
                    f"Requested room "
                    f"{self.requested_room} "
                    f"identified as "
                    f"{target_room_id}. "
                    f"Navigating there."
                )

                self.go_to_final_room(
                    target_room_id
                )

                return

            self.publish_status(
                "Requested room could not be identified."
            )

        self.publish_status(
            "Exploration finished."
        )

        self.running = False

    # =============================================================
    # FINAL ROOM NAVIGATION
    # =============================================================

    def go_to_final_room(self, room_id):

        points = self.rooms[
            room_id
        ]['points']

        if not points:

            self.publish_status(
                "Requested room has no navigation point."
            )

            self.running = False
            return

        point = points[0]

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = float(
            point['x']
        )

        goal_msg.pose.pose.position.y = float(
            point['y']
        )

        yaw = float(
            point.get('yaw', 0.0)
        )

        goal_msg.pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )

        goal_msg.pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        future = self.nav_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self.final_goal_response
        )

    # =============================================================
    # FINAL GOAL RESPONSE
    # =============================================================

    def final_goal_response(self, future):

        goal_handle = future.result()

        if goal_handle is None:

            self.publish_status(
                "Final navigation returned no goal handle."
            )

            self.running = False
            return

        if not goal_handle.accepted:

            self.publish_status(
                "Final navigation goal rejected."
            )

            self.running = False
            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.final_goal_result
        )

    # =============================================================
    # FINAL GOAL RESULT
    # =============================================================

    def final_goal_result(self, future):

        result = future.result()

        if result is not None and \
                result.status == GoalStatus.STATUS_SUCCEEDED:

            self.publish_status(
                f"Reached requested room: "
                f"{self.requested_room}"
            )

            report_msg = String()

            report_msg.data = (
                f"The requested room is "
                f"{self.requested_room}."
            )

            self.report_pub.publish(
                report_msg
            )

        else:

            self.publish_status(
                "Could not reach requested room."
            )

        self.publish_status(
            "AUTONOMOUS RUN FINISHED."
        )

        self.running = False
        self.finishing = False


def main(args=None):

    rclpy.init(args=args)

    node = RoomExplorerNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()