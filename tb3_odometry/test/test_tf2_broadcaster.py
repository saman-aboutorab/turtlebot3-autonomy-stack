"""
test_tf2_broadcaster.py
=======================
Unit tests for TF2Broadcaster.

Tests focus on the _odom_to_transform() conversion — the only logic
this node contains. The actual TF broadcast is ROS2 middleware and
does not need to be tested here.

Run with:
  cd ~/turtlebot3-autonomy-stack
  source install/setup.bash
  python3 -m pytest tb3_odometry/test/test_tf2_broadcaster.py -v
"""

import math
import pytest

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion


# ── helpers ────────────────────────────────────────────────────────────────

def make_odometry(x=0.0, y=0.0, yaw=0.0, sec=0) -> Odometry:
    """Build an Odometry message with the given pose."""
    msg = Odometry()
    msg.header.stamp    = Time(sec=sec, nanosec=0)
    msg.header.frame_id = 'odom'
    msg.child_frame_id  = 'base_footprint'
    msg.pose.pose.position    = Point(x=x, y=y, z=0.0)
    msg.pose.pose.orientation = Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )
    return msg


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_init):
    from tb3_odometry.tf2_broadcaster import TF2Broadcaster
    n = TF2Broadcaster()
    yield n
    n.destroy_node()


# ── tests ──────────────────────────────────────────────────────────────────

class TestOdomToTransform:

    def test_frame_ids_copied_correctly(self, node):
        """parent=odom, child=base_footprint must be preserved."""
        odom = make_odometry()
        t = node._odom_to_transform(odom)
        assert t.header.frame_id == 'odom'
        assert t.child_frame_id  == 'base_footprint'

    def test_timestamp_taken_from_odom_not_now(self, node):
        """Timestamp must come from the odom message, not the system clock."""
        odom = make_odometry(sec=42)
        t = node._odom_to_transform(odom)
        assert t.header.stamp.sec == 42

    def test_translation_xy_copied(self, node):
        """x and y must be copied from odom position."""
        odom = make_odometry(x=1.5, y=-0.75)
        t = node._odom_to_transform(odom)
        assert t.transform.translation.x == pytest.approx(1.5)
        assert t.transform.translation.y == pytest.approx(-0.75)

    def test_translation_z_always_zero(self, node):
        """base_footprint is always on the floor — z must be 0."""
        odom = make_odometry(x=0.0, y=0.0)
        t = node._odom_to_transform(odom)
        assert t.transform.translation.z == 0.0

    def test_rotation_quaternion_copied(self, node):
        """Quaternion from odom orientation must appear in transform rotation."""
        yaw = math.pi / 4  # 45 degrees
        odom = make_odometry(yaw=yaw)
        t = node._odom_to_transform(odom)

        expected_z = math.sin(yaw / 2.0)
        expected_w = math.cos(yaw / 2.0)
        assert t.transform.rotation.x == pytest.approx(0.0)
        assert t.transform.rotation.y == pytest.approx(0.0)
        assert t.transform.rotation.z == pytest.approx(expected_z, abs=1e-6)
        assert t.transform.rotation.w == pytest.approx(expected_w, abs=1e-6)

    def test_zero_pose_is_identity_transform(self, node):
        """At x=0,y=0,yaw=0, the transform should be the identity."""
        odom = make_odometry(x=0.0, y=0.0, yaw=0.0)
        t = node._odom_to_transform(odom)
        assert t.transform.translation.x == pytest.approx(0.0)
        assert t.transform.translation.y == pytest.approx(0.0)
        assert t.transform.translation.z == pytest.approx(0.0)
        assert t.transform.rotation.z    == pytest.approx(0.0)
        assert t.transform.rotation.w    == pytest.approx(1.0)

    def test_180_degree_rotation(self, node):
        """Yaw of pi radians → robot facing backward."""
        odom = make_odometry(yaw=math.pi)
        t = node._odom_to_transform(odom)
        # qz = sin(pi/2) = 1.0, qw = cos(pi/2) ≈ 0.0
        assert t.transform.rotation.z == pytest.approx(1.0, abs=1e-6)
        assert t.transform.rotation.w == pytest.approx(0.0, abs=1e-6)
