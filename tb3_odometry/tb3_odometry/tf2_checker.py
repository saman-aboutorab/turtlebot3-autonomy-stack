#!/usr/bin/env python3
"""
tf2_checker.py
==============
A diagnostic Python node that reads the TF2 transform tree and prints
every expected transform for Stage 1. Run this after description.launch.py
to verify the URDF loaded correctly.

This node teaches you:
  - How to query TF2 from Python (tf2_ros.Buffer + TransformListener)
  - What the transform data looks like (translation + rotation as quaternion)
  - How to detect broken or missing transforms

DATA FLOW:
  robot_state_publisher (C++ node)
    → publishes to /tf_static topic (latched, always available)
    → tf2_ros.Buffer caches all incoming /tf and /tf_static messages
    → tf2_ros.TransformListener subscribes to /tf and /tf_static
    → we call buffer.lookup_transform() to query any frame pair

USAGE (laptop, no robot needed):
  Terminal 1: ros2 launch tb3_bringup description.launch.py
  Terminal 2: ros2 run tb3_odometry tf2_checker
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from tf2_ros import TransformException


# All transforms we expect after the URDF loads.
# Format: (parent_frame, child_frame, description)
EXPECTED_TRANSFORMS = [
    ('base_footprint', 'base_link',        'Floor plane → robot body'),
    ('base_link',      'wheel_left_link',  'Robot body → left wheel'),
    ('base_link',      'wheel_right_link', 'Robot body → right wheel'),
    ('base_link',      'caster_back_link', 'Robot body → rear caster'),
    ('base_link',      'imu_link',         'Robot body → IMU chip'),
    ('base_link',      'base_scan',        'Robot body → LiDAR scan origin'),
]


class TF2Checker(Node):

    def __init__(self):
        super().__init__('tf2_checker')

        # tf2_ros.Buffer stores all incoming TF messages in a time-indexed cache.
        # cache_time controls how far back in history it keeps (default 10s).
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))

        # TransformListener subscribes to /tf and /tf_static and feeds them
        # into the Buffer automatically. You never call it directly — you just
        # create it and it runs in the background.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Wait a moment for the buffer to fill, then check once and exit.
        # Using a one-shot timer (period = 2.0s, fires once).
        self.timer = self.create_timer(2.0, self.check_transforms)
        self._checked = False

        self.get_logger().info(
            'TF2 checker started — waiting 2 seconds for TF buffer to fill...'
        )

    def check_transforms(self):
        """Query every expected transform and report pass/fail."""

        if self._checked:
            return
        self._checked = True
        self.timer.cancel()

        self.get_logger().info('=' * 60)
        self.get_logger().info('TF2 TRANSFORM CHECK — Stage 1 URDF Verification')
        self.get_logger().info('=' * 60)

        all_ok = True

        for parent, child, description in EXPECTED_TRANSFORMS:
            try:
                # lookup_transform(target, source, time)
                # time = rclpy.time.Time() means "latest available transform"
                transform = self.tf_buffer.lookup_transform(
                    target_frame=parent,
                    source_frame=child,
                    time=rclpy.time.Time(),
                )

                t = transform.transform.translation
                r = transform.transform.rotation

                self.get_logger().info(
                    f'\n  [PASS] {parent} → {child}\n'
                    f'         {description}\n'
                    f'         translation: x={t.x:.4f}  y={t.y:.4f}  z={t.z:.4f}\n'
                    f'         rotation quat: x={r.x:.3f} y={r.y:.3f} '
                    f'z={r.z:.3f} w={r.w:.3f}'
                )

            except TransformException as e:
                all_ok = False
                self.get_logger().error(
                    f'\n  [FAIL] {parent} → {child}\n'
                    f'         {description}\n'
                    f'         Error: {e}'
                )

        self.get_logger().info('=' * 60)
        if all_ok:
            self.get_logger().info(
                'ALL TRANSFORMS OK — URDF loaded correctly.\n'
                'Key check: base_link → base_scan z should be ~0.172m\n'
                '           base_link → imu_link  z should be ~0.068m'
            )
        else:
            self.get_logger().error(
                'SOME TRANSFORMS MISSING — check that description.launch.py '
                'is running and the xacro file compiled without errors.'
            )
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = TF2Checker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
