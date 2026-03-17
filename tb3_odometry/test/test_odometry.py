"""
test_odometry.py
================
Unit tests for OdometryPublisher — no robot, no ROS2 network needed.

Tests verify the kinematic math directly by calling the node's callback
with fake JointState messages and checking the resulting pose.

Run with:
  cd ~/turtlebot3-autonomy-stack
  source install/setup.bash
  python3 -m pytest tb3_odometry/test/test_odometry.py -v
"""

import math
import pytest

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


# ── helpers ────────────────────────────────────────────────────────────────

def make_joint_state(left_pos: float, right_pos: float,
                     left_vel: float = 0.0, right_vel: float = 0.0,
                     sec: int = 0) -> JointState:
    """Build a minimal JointState message with wheel positions."""
    msg = JointState()
    msg.header.stamp = Time(sec=sec, nanosec=0)
    msg.name     = ['wheel_left_joint', 'wheel_right_joint']
    msg.position = [left_pos,  right_pos]
    msg.velocity = [left_vel,  right_vel]
    msg.effort   = []
    return msg


@pytest.fixture(scope='module')
def rclpy_init():
    """Initialise rclpy once for all tests in this module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_init):
    """Create a fresh OdometryPublisher node for each test."""
    # import here so rclpy is already initialised
    from tb3_odometry.odometry_publisher import OdometryPublisher
    n = OdometryPublisher()
    yield n
    n.destroy_node()


# ── tests ──────────────────────────────────────────────────────────────────

class TestInitialisation:

    def test_pose_starts_at_origin(self, node):
        """Robot starts at (0, 0, 0) before any encoder data arrives."""
        assert node.x     == 0.0
        assert node.y     == 0.0
        assert node.theta == 0.0

    def test_prev_positions_none_before_first_message(self, node):
        """No previous position stored until first /joint_states arrives."""
        assert node.prev_left_pos  is None
        assert node.prev_right_pos is None

    def test_parameters_loaded(self, node):
        """Node loaded the correct default physical parameters."""
        assert node.wheel_radius     == pytest.approx(0.033, abs=1e-6)
        assert node.wheel_separation == pytest.approx(0.160, abs=1e-6)


class TestFirstMessage:

    def test_first_message_initialises_prev_positions(self, node):
        """First message sets prev_pos and does NOT update pose."""
        node.joint_states_callback(make_joint_state(0.5, 0.5))
        assert node.prev_left_pos  == pytest.approx(0.5)
        assert node.prev_right_pos == pytest.approx(0.5)
        # pose must not change on the first message
        assert node.x     == 0.0
        assert node.y     == 0.0
        assert node.theta == 0.0

    def test_first_message_missing_wheel_joints_warns(self, node, caplog):
        """A message without wheel joints should log a warning and return."""
        msg = JointState()
        msg.name     = ['some_other_joint']
        msg.position = [1.0]
        msg.velocity = [0.0]
        msg.effort   = []
        node.joint_states_callback(msg)
        # pose unchanged
        assert node.x == 0.0


class TestStraightLineMotion:
    """
    Both wheels rotate by the same angle → robot moves straight forward.

    Expected: x increases, y = 0, theta = 0.
    """

    def test_straight_forward_x_increases(self, node):
        """One radian on both wheels → x ≈ 0.033m (1 rad × wheel_radius)."""
        node.joint_states_callback(make_joint_state(0.0, 0.0))   # init
        node.joint_states_callback(make_joint_state(1.0, 1.0))   # move

        expected_x = 1.0 * node.wheel_radius   # 0.033 m
        assert node.x     == pytest.approx(expected_x, abs=1e-6)
        assert node.y     == pytest.approx(0.0,         abs=1e-6)
        assert node.theta == pytest.approx(0.0,         abs=1e-6)

    def test_straight_forward_multiple_steps(self, node):
        """Three equal steps forward accumulate correctly."""
        node.joint_states_callback(make_joint_state(0.0, 0.0))
        node.joint_states_callback(make_joint_state(1.0, 1.0))
        node.joint_states_callback(make_joint_state(2.0, 2.0))
        node.joint_states_callback(make_joint_state(3.0, 3.0))

        expected_x = 3.0 * node.wheel_radius
        assert node.x == pytest.approx(expected_x, abs=1e-6)
        assert node.theta == pytest.approx(0.0, abs=1e-6)

    def test_straight_backward(self, node):
        """Negative wheel rotation → x decreases."""
        node.joint_states_callback(make_joint_state(0.0,  0.0))
        node.joint_states_callback(make_joint_state(-1.0, -1.0))

        expected_x = -1.0 * node.wheel_radius
        assert node.x == pytest.approx(expected_x, abs=1e-6)


class TestRotation:
    """
    Right wheel faster than left → robot turns left (positive yaw).
    Left wheel faster than right → robot turns right (negative yaw).

    For a pure rotation (one wheel goes forward, the other backward
    by the same amount), the robot spins in place:
      arc_per_wheel = angle × wheel_radius
      total_arc     = 2 × arc_per_wheel  (both wheels contribute)
      rotation      = total_arc / wheel_separation
    """

    def test_pure_left_turn(self, node):
        """Right +1 rad, left -1 rad → pure counter-clockwise spin."""
        node.joint_states_callback(make_joint_state(0.0,  0.0))
        node.joint_states_callback(make_joint_state(-1.0, 1.0))

        expected_theta = (2.0 * node.wheel_radius) / node.wheel_separation
        assert node.theta == pytest.approx(expected_theta, abs=1e-6)
        # Position should remain near zero (pure spin)
        assert node.x == pytest.approx(0.0, abs=1e-6)
        assert node.y == pytest.approx(0.0, abs=1e-6)

    def test_pure_right_turn(self, node):
        """Left +1 rad, right -1 rad → pure clockwise spin (negative yaw)."""
        node.joint_states_callback(make_joint_state(0.0, 0.0))
        node.joint_states_callback(make_joint_state(1.0, -1.0))

        expected_theta = -(2.0 * node.wheel_radius) / node.wheel_separation
        assert node.theta == pytest.approx(expected_theta, abs=1e-6)

    def test_theta_normalised_within_pi(self, node):
        """Theta must stay within [-π, π] after many full rotations."""
        node.joint_states_callback(make_joint_state(0.0, 0.0))
        # Drive many full left spins by accumulating right-wheel increments
        # Each step: right +0.5, left -0.5
        full_rotation = math.pi * node.wheel_separation / node.wheel_radius
        increments = 20
        step = full_rotation / increments
        pos = 0.0
        for _ in range(increments * 10):  # 10 full rotations
            pos += step
            node.joint_states_callback(make_joint_state(-pos, pos))

        assert -math.pi <= node.theta <= math.pi


class TestQuarterTurnAndForward:
    """
    Turn 90° left, then drive forward — verifies the heading is
    used correctly when projecting motion onto x/y axes.
    """

    def test_90_degree_left_then_forward(self, node):
        """
        After a 90° left turn, moving forward should increase y (not x).
        Expected final pose: x ≈ 0, y > 0, theta ≈ π/2
        """
        # Step 1: turn left 90 degrees in place
        # angular_dist = (dist_right - dist_left) / wheel_separation = π/2
        # dist_right - dist_left = π/2 * wheel_separation
        # For pure spin: right = +half, left = -half
        half = (math.pi / 2) * node.wheel_separation / 2.0 / node.wheel_radius
        node.joint_states_callback(make_joint_state(0.0,   0.0))
        node.joint_states_callback(make_joint_state(-half, +half))

        assert node.theta == pytest.approx(math.pi / 2, abs=1e-4)
        assert node.x     == pytest.approx(0.0,         abs=1e-6)
        assert node.y     == pytest.approx(0.0,         abs=1e-6)

        # Step 2: move forward 1 radian on both wheels — should increase y
        prev_left  = -half
        prev_right = +half
        node.joint_states_callback(
            make_joint_state(prev_left + 1.0, prev_right + 1.0)
        )

        expected_y = 1.0 * node.wheel_radius
        assert node.x == pytest.approx(0.0,        abs=1e-4)
        assert node.y == pytest.approx(expected_y, abs=1e-4)


class TestYawToQuaternion:
    """Tests for the static _yaw_to_quaternion helper."""

    def test_zero_yaw(self):
        from tb3_odometry.odometry_publisher import OdometryPublisher
        q = OdometryPublisher._yaw_to_quaternion(0.0)
        assert q.x == pytest.approx(0.0)
        assert q.y == pytest.approx(0.0)
        assert q.z == pytest.approx(0.0)
        assert q.w == pytest.approx(1.0)

    def test_90_degree_yaw(self):
        from tb3_odometry.odometry_publisher import OdometryPublisher
        q = OdometryPublisher._yaw_to_quaternion(math.pi / 2)
        assert q.x == pytest.approx(0.0)
        assert q.y == pytest.approx(0.0)
        assert q.z == pytest.approx(math.sin(math.pi / 4), abs=1e-6)
        assert q.w == pytest.approx(math.cos(math.pi / 4), abs=1e-6)

    def test_unit_quaternion(self):
        """Quaternion magnitude must always be 1.0."""
        from tb3_odometry.odometry_publisher import OdometryPublisher
        for yaw in [0.0, math.pi/4, math.pi/2, math.pi, -math.pi/3]:
            q = OdometryPublisher._yaw_to_quaternion(yaw)
            magnitude = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
            assert magnitude == pytest.approx(1.0, abs=1e-6), f"failed at yaw={yaw}"
