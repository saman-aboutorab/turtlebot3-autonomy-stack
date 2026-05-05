#!/usr/bin/env python3
"""
nav_goal_sender.py
==================

Sends a single NavigateToPose goal to Nav2 and reports success or failure.

This node demonstrates the ROS2 action client API with Nav2. A goal pose is
expressed in the map frame (x, y, yaw) and sent to the bt_navigator action
server, which handles planning, execution, and recovery autonomously.

HOW ROS2 ACTIONS WORK:
  An action is like a service, but designed for long-running tasks:
    1. Client sends a goal → server responds "accepted" or "rejected" immediately
    2. Server sends periodic feedback while executing (current pose, ETA)
    3. Server sends a final result when done (success or failure code)

  This is the right abstraction for navigation: you don't want to block the
  entire process waiting for the robot to arrive (could take 30+ seconds),
  but you do want to know when it's done. Actions give you non-blocking
  execution with result notification.

DATA FLOW:
  nav_goal_sender.py
       │  NavigateToPose goal (map frame pose)
       ▼
  bt_navigator (Nav2)
       │  ComputePathToPose ──► planner_server ──► global path
       │  FollowPath        ──► controller_server ──► /cmd_vel_raw
       │  Recovery          ──► spin / back up / re-plan if stuck
       ▼
  Result: SUCCESS (status=4) or FAILURE (status=6)

PARAMETERS:
  goal_x   (float, default: 1.0)  X coordinate in map frame (metres)
  goal_y   (float, default: 0.0)  Y coordinate in map frame (metres)
  goal_yaw (float, default: 0.0)  target heading (radians, 0 = facing +X axis)

USAGE:
  # Navigate to (1.0, 0.5) facing forward
  ros2 run tb3_navigation nav_goal_sender.py \\
    --ros-args -p goal_x:=1.0 -p goal_y:=0.5 -p goal_yaw:=0.0

  # Navigate to (-0.5, 1.0) facing left (~90 degrees)
  ros2 run tb3_navigation nav_goal_sender.py \\
    --ros-args -p goal_x:=-0.5 -p goal_y:=1.0 -p goal_yaw:=1.5708
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float) -> dict:
    """Convert a yaw angle (radians) to quaternion components.

    ROS uses quaternions to represent 3D orientation. For a 2D robot rotating
    only around the Z axis (yaw), the conversion is:
      x = 0, y = 0, z = sin(yaw/2), w = cos(yaw/2)
    """
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0),
    }


class NavGoalSender(Node):

    def __init__(self):
        super().__init__('nav_goal_sender')

        self.declare_parameter('goal_x',   1.0)
        self.declare_parameter('goal_y',   0.0)
        self.declare_parameter('goal_yaw', 0.0)

        # ActionClient(node, action_type, action_name)
        # 'navigate_to_pose' is the name bt_navigator advertises.
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self):
        goal_x   = self.get_parameter('goal_x').value
        goal_y   = self.get_parameter('goal_y').value
        goal_yaw = self.get_parameter('goal_yaw').value

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._client.wait_for_server()

        # Build goal pose in the map frame
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0

        q = yaw_to_quaternion(goal_yaw)
        goal_pose.pose.orientation.x = q['x']
        goal_pose.pose.orientation.y = q['y']
        goal_pose.pose.orientation.z = q['z']
        goal_pose.pose.orientation.w = q['w']

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(
            f'Sending goal → x={goal_x:.2f} m  y={goal_y:.2f} m  yaw={goal_yaw:.2f} rad'
        )

        # send_goal_async: returns immediately with a future.
        # The future resolves when the server accepts or rejects the goal.
        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 — is the map loaded?')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted — robot is navigating...')

        # get_result_async: returns a future that resolves when navigation finishes.
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        pos = feedback_msg.feedback.current_pose.pose.position
        # throttle_duration_sec avoids flooding the terminal at 20 Hz
        self.get_logger().info(
            f'Current position: x={pos.x:.2f}  y={pos.y:.2f}',
            throttle_duration_sec=2.0,
        )

    def _result_callback(self, future):
        result = future.result()
        status = result.status

        # GoalStatus codes: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrived at goal successfully.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation was cancelled.')
        else:
            self.get_logger().error(
                f'Navigation failed (status={status}). '
                'Check if AMCL is localised and the goal is reachable.'
            )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSender()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
