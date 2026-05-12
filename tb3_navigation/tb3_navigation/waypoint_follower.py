#!/usr/bin/env python3
"""
waypoint_follower.py
====================

Sends a sequence of waypoints to Nav2's FollowWaypoints action server.
The robot visits each pose in order, autonomously, without any human input
between waypoints.

HOW FollowWaypoints DIFFERS FROM NavigateToPose:
  NavigateToPose (Step 10): one goal at a time — you send the next goal
    only after the previous one completes.
  FollowWaypoints (Step 11): you send ALL goals at once in a single action
    call. The nav2_waypoint_follower node chains them internally, calling
    NavigateToPose for each in sequence. You get per-waypoint feedback and
    a final list of any waypoints that were missed.

DATA FLOW:
  waypoint_follower.py
       │  FollowWaypoints action  (list of PoseStamped)
       ▼
  nav2_waypoint_follower (lifecycle node, already running in navigation.launch.py)
       │  NavigateToPose  ← one at a time, internally
       ▼
  bt_navigator → planner_server → controller_server → /cmd_vel_raw → motors

WAYPOINTS (my_room_v3 map frame):
  These were recorded with the Publish Point tool in RViz.
  Edit WAYPOINTS below to change the mission route.

USAGE:
  ros2 run tb3_navigation waypoint_follower.py
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


# ── Mission waypoints ─────────────────────────────────────────────────────────
# Each entry: (x, y, yaw)
#   x, y  — position in the map frame (metres), recorded with Publish Point
#   yaw   — target heading (radians). 0.0 = facing map +x axis.
#            Use 0.0 if you don't care about final heading at each stop.
WAYPOINTS = [
    (-1.876, -1.082, 0.0),  # waypoint 1
    (-3.015,  0.924, 0.0),  # waypoint 2
    ( 0.324, -0.078, 0.0),  # waypoint 3
]


def make_pose(x: float, y: float, yaw: float, clock) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = clock.now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def send_waypoints(self):
        self.get_logger().info('Waiting for follow_waypoints action server...')
        self._client.wait_for_server()

        poses = [make_pose(x, y, yaw, self.get_clock()) for x, y, yaw in WAYPOINTS]

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.get_logger().info(f'Sending {len(poses)} waypoints...')
        for i, (x, y, _) in enumerate(WAYPOINTS):
            self.get_logger().info(f'  [{i + 1}] x={x:.3f}  y={y:.3f}')

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected — is Nav2 running and localised?')
            rclpy.shutdown()
            return

        self.get_logger().info('Mission accepted — robot is navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        idx = feedback_msg.feedback.current_waypoint
        total = len(WAYPOINTS)
        self.get_logger().info(
            f'Navigating to waypoint {idx + 1} / {total}',
            throttle_duration_sec=3.0,
        )

    def _result_callback(self, future):
        result = future.result()
        status = result.status
        missed = list(result.result.missed_waypoints)

        if status == GoalStatus.STATUS_SUCCEEDED and not missed:
            self.get_logger().info(
                f'Mission complete — all {len(WAYPOINTS)} waypoints reached.'
            )
        elif missed:
            reached = len(WAYPOINTS) - len(missed)
            self.get_logger().warn(
                f'Mission finished: {reached}/{len(WAYPOINTS)} waypoints reached. '
                f'Missed indices: {missed}'
            )
        else:
            self.get_logger().error(f'Mission failed (status={status}).')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    node.send_waypoints()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
