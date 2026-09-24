#!/usr/bin/env python3

from math import sin, cos, pi
import sys
import random

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion


# ============================================================
# MAP BOUNDARIES
# ============================================================
# Adjust these values to the boundaries of YOUR map.
# You can find them in RViz using "Publish Point"
# and:
#
# ros2 topic echo /clicked_point
#
# Make sure the points are given in the "map" frame.

map_min_x = -1.0
map_max_x = 4.6

map_min_y = -2.9
map_max_y = 2.4


# ============================================================
# GLOBAL VARIABLES
# ============================================================

success = True


# ============================================================
# MAIN
# ============================================================

def main():
    global auto_chaos
    global nav_to_pose_client

    rclpy.init()

    # Create ROS2 node
    auto_chaos = rclpy.create_node('auto_goals')

    # Create Action Client for Nav2
    nav_to_pose_client = ActionClient(
        auto_chaos,
        NavigateToPose,
        'navigate_to_pose'
    )

    # --------------------------------------------------------
    # Wait for Nav2 action server
    # --------------------------------------------------------

    while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
        print("Server still not available; waiting...")

    print("NavigateToPose action server available!")
    print("Starting autonomous exploration...")

    # --------------------------------------------------------
    # Continuously generate and send random goals
    # --------------------------------------------------------

    while rclpy.ok():

        try:
            # Generate random position inside map boundaries
            position = generatePosition()

            # Generate random orientation
            orientation = generateOrientation()

            # Send navigation goal
            goal_handle = sendGoal(position, orientation)

            # Wait for result
            status = checkResult(goal_handle)

            # Print result
            if status == GoalStatus.STATUS_SUCCEEDED:
                print("Goal reached successfully!")
            else:
                print("Goal finished with status:", status)

        except KeyboardInterrupt:
            print("Shutdown requested... complying...")
            break

    # --------------------------------------------------------
    # Shutdown
    # --------------------------------------------------------

    nav_to_pose_client.destroy()
    auto_chaos.destroy_node()
    rclpy.shutdown()


# ============================================================
# SEND GOAL
# ============================================================

def sendGoal(position, orientation):
    global auto_chaos
    global nav_to_pose_client

    # Create NavigateToPose goal
    goal = NavigateToPose.Goal()

    # --------------------------------------------------------
    # IMPORTANT:
    # "map" means that this is an ABSOLUTE goal.
    #
    # The x/y coordinates are interpreted relative to
    # the global map frame.
    # --------------------------------------------------------

    goal.pose.header.frame_id = "map"

    # Add current timestamp
    goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()

    # Set position and orientation
    goal.pose.pose.position = position
    goal.pose.pose.orientation = orientation

    print()
    print("----------------------------------------")
    print("New navigation goal:")
    print(
        "X: {:.2f} m".format(
            goal.pose.pose.position.x
        )
    )
    print(
        "Y: {:.2f} m".format(
            goal.pose.pose.position.y
        )
    )
    print("----------------------------------------")

    # --------------------------------------------------------
    # Send goal asynchronously
    # --------------------------------------------------------

    send_goal_future = nav_to_pose_client.send_goal_async(goal)

    # Wait until the goal has been accepted/rejected
    rclpy.spin_until_future_complete(
        auto_chaos,
        send_goal_future
    )

    goal_handle = send_goal_future.result()

    # --------------------------------------------------------
    # Check if goal was accepted
    # --------------------------------------------------------

    if not goal_handle.accepted:

        print("Goal was rejected!")

        nav_to_pose_client.destroy()
        auto_chaos.destroy_node()
        rclpy.shutdown()

        sys.exit(0)

    print("Goal accepted!")

    return goal_handle


# ============================================================
# CHECK RESULT
# ============================================================

def checkResult(goal_handle):

    # Request result from action server
    get_result_future = goal_handle.get_result_async()

    # Wait until navigation is finished
    rclpy.spin_until_future_complete(
        auto_chaos,
        get_result_future
    )

    # Get navigation status
    status = get_result_future.result().status

    # --------------------------------------------------------
    # Interpret result
    # --------------------------------------------------------

    if status == GoalStatus.STATUS_SUCCEEDED:
        print("Reached goal!")

    elif status == GoalStatus.STATUS_ABORTED:
        print("Navigation aborted!")

    elif status == GoalStatus.STATUS_CANCELED:
        print("Navigation canceled!")

    else:
        print("Navigation finished with status:", status)

    return status


# ============================================================
# GENERATE RANDOM POSITION
# ============================================================

def generatePosition():

    position = Point()

    # Generate random X coordinate
    position.x = round(
        random.uniform(
            map_min_x,
            map_max_x
        ),
        2
    )

    # Generate random Y coordinate
    position.y = round(
        random.uniform(
            map_min_y,
            map_max_y
        ),
        2
    )

    # Robot operates in 2D
    position.z = 0.0

    return position


# ============================================================
# GENERATE RANDOM ORIENTATION
# ============================================================

def generateOrientation():

    quat = Quaternion()

    # Generate random yaw angle
    # between -180° and +180°
    yaw = random.uniform(
        -pi,
        pi
    )

    # Convert yaw angle to quaternion
    #
    # For a planar robot:
    # x = 0
    # y = 0
    # z = sin(yaw/2)
    # w = cos(yaw/2)

    quat.x = 0.0
    quat.y = 0.0

    quat.z = sin(
        yaw / 2.0
    )

    quat.w = cos(
        yaw / 2.0
    )

    return quat


# ============================================================
# START PROGRAM
# ============================================================

if __name__ == '__main__':
    main()