#!/usr/bin/env python3
"""
tf2_broadcaster.py
==================
Subscribes to /odom (nav_msgs/Odometry) and broadcasts the dynamic TF2
transform odom -> base_footprint.

WHY THIS IS A SEPARATE NODE FROM odometry_publisher.py:
  The odometry publisher computes the pose and publishes it as a ROS2 message.
  This node bridges that message into the TF2 system.
  Keeping them separate means in Step 6 we can swap to broadcasting from the
  EKF filtered output (/odometry/filtered) instead of raw /odom — without
  touching the odometry publisher at all.

STATIC vs DYNAMIC TRANSFORMS:
  - Static (robot_state_publisher, Step 2): frames that never move relative
    to each other (base_link -> base_scan). Broadcast once to /tf_static.
  - Dynamic (this node): frames that move over time (odom -> base_footprint).
    Broadcast continuously to /tf on every /odom message.

THE TRANSFORM WE BROADCAST:
  parent frame: odom          (the fixed world reference frame)
  child frame:  base_footprint (the robot's floor-level frame)
  value:        the robot's current x, y, yaw from /odom

SUBSCRIPTIONS:
  /odom     nav_msgs/Odometry   — robot pose from odometry_publisher.py

PUBLICATIONS (via TF2, not a regular publisher):
  /tf       tf2_msgs/TFMessage  — dynamic transform odom -> base_footprint
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class TF2Broadcaster(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')

        # ----------------------------------------------------------------
        # PARAMETERS
        # These must match the frame IDs used by odometry_publisher.py.
        # ----------------------------------------------------------------
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        # ----------------------------------------------------------------
        # TRANSFORM BROADCASTER
        # tf2_ros.TransformBroadcaster publishes TransformStamped messages
        # to the /tf topic. Any node with a tf2_ros.Buffer + TransformListener
        # will receive and cache them automatically.
        #
        # Note: this is NOT a regular ROS2 publisher — you don't create it
        # with self.create_publisher(). It manages its own internal publisher.
        # ----------------------------------------------------------------
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # ----------------------------------------------------------------
        # SUBSCRIBER — /odom
        # BEST_EFFORT QoS must match the publisher in odometry_publisher.py.
        # ----------------------------------------------------------------
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos
        )

        self.get_logger().info(
            f'TF2Broadcaster started.\n'
            f'  subscribing to: /odom\n'
            f'  broadcasting:   {self.odom_frame_id} -> {self.base_frame_id} on /tf'
        )

    def odom_callback(self, msg: Odometry):
        """
        Receives Odometry and broadcasts the corresponding TF transform.

        The Odometry message already contains the pose in the correct format.
        We just need to repack it into a TransformStamped and send it.

        msg.header.frame_id      = 'odom'          (parent frame)
        msg.child_frame_id       = 'base_footprint' (child frame)
        msg.pose.pose.position   = (x, y, z)
        msg.pose.pose.orientation = quaternion
        """
        transform = self._odom_to_transform(msg)
        self.broadcaster.sendTransform(transform)

    def _odom_to_transform(self, odom: Odometry) -> TransformStamped:
        """
        Converts a nav_msgs/Odometry message into a geometry_msgs/TransformStamped.

        This is a pure data conversion — no math, no state.
        Extracted as a separate method so unit tests can call it directly
        without needing a live ROS2 broadcaster.

        A TransformStamped encodes:
          "at time T, frame CHILD is located at TRANSLATION with ROTATION
           relative to frame PARENT"

        The Odometry message encodes exactly the same information, just in a
        different message type. We copy fields across.
        """
        t = TransformStamped()

        # --- Header ---
        # Use the timestamp from the incoming odometry message, not now().
        # This keeps the TF timestamp in sync with when the pose was computed,
        # which matters for TF lookups that request a specific time.
        t.header.stamp    = odom.header.stamp
        t.header.frame_id = odom.header.frame_id       # 'odom'
        t.child_frame_id  = odom.child_frame_id        # 'base_footprint'

        # --- Translation ---
        # Copy the robot's x, y position.
        # z = 0 because base_footprint is always at floor level.
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0

        # --- Rotation ---
        # Copy the quaternion directly — odometry_publisher already converted
        # yaw to quaternion. No conversion needed here.
        t.transform.rotation = odom.pose.pose.orientation

        return t


def main(args=None):
    rclpy.init(args=args)
    node = TF2Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
