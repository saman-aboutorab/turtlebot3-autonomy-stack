"""
test_ekf_node.py
================
Unit tests for EKFNode.

Tests focus on the pure-math methods:
  _predict()             — motion model and covariance propagation
  _correct()             — Kalman gain and state/covariance update
  _yaw_from_quaternion() — quaternion → yaw extraction
  _yaw_to_quaternion()   — yaw → quaternion conversion

These methods have no ROS2 I/O, so tests can call them directly without
needing a live publisher, subscriber, or spin loop.

Run with:
  cd ~/turtlebot3-autonomy-stack
  source install/setup.bash
  python3 -m pytest tb3_odometry/test/test_ekf_node.py -v
"""

import math

import numpy as np
import pytest

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion


# ── fixtures ───────────────────────────────────────────────────────────────

@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_init):
    from tb3_odometry.ekf_node import EKFNode
    n = EKFNode()
    yield n
    n.destroy_node()


# ── helpers ────────────────────────────────────────────────────────────────

def make_quaternion(yaw: float) -> Quaternion:
    """Build a pure-yaw quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def fresh_state():
    """3-element zero state and 3×3 identity covariance."""
    return np.zeros(3), np.eye(3)


# ── TestYawFromQuaternion ──────────────────────────────────────────────────

class TestYawFromQuaternion:
    """Static helper: quaternion → yaw."""

    def test_zero_yaw(self):
        from tb3_odometry.ekf_node import EKFNode
        q = make_quaternion(0.0)
        assert EKFNode._yaw_from_quaternion(q) == pytest.approx(0.0)

    def test_90_degrees(self):
        from tb3_odometry.ekf_node import EKFNode
        q = make_quaternion(math.pi / 2)
        assert EKFNode._yaw_from_quaternion(q) == pytest.approx(math.pi / 2)

    def test_minus_90_degrees(self):
        from tb3_odometry.ekf_node import EKFNode
        q = make_quaternion(-math.pi / 2)
        assert EKFNode._yaw_from_quaternion(q) == pytest.approx(-math.pi / 2)

    def test_180_degrees(self):
        from tb3_odometry.ekf_node import EKFNode
        q = make_quaternion(math.pi)
        # atan2 returns ±π — accept either
        result = EKFNode._yaw_from_quaternion(q)
        assert abs(result) == pytest.approx(math.pi, abs=1e-6)

    def test_roundtrip_arbitrary_angle(self):
        """yaw → quaternion → yaw must return the original angle."""
        from tb3_odometry.ekf_node import EKFNode
        for yaw in [0.3, -1.2, 2.8, -3.0]:
            q      = EKFNode._yaw_to_quaternion(yaw)
            result = EKFNode._yaw_from_quaternion(q)
            assert result == pytest.approx(yaw, abs=1e-9)


# ── TestYawToQuaternion ────────────────────────────────────────────────────

class TestYawToQuaternion:

    def test_zero_yaw_is_identity(self):
        from tb3_odometry.ekf_node import EKFNode
        q = EKFNode._yaw_to_quaternion(0.0)
        assert q.x == pytest.approx(0.0)
        assert q.y == pytest.approx(0.0)
        assert q.z == pytest.approx(0.0)
        assert q.w == pytest.approx(1.0)

    def test_90_degrees(self):
        from tb3_odometry.ekf_node import EKFNode
        q = EKFNode._yaw_to_quaternion(math.pi / 2)
        assert q.z == pytest.approx(math.sin(math.pi / 4), abs=1e-9)
        assert q.w == pytest.approx(math.cos(math.pi / 4), abs=1e-9)

    def test_unit_magnitude(self):
        """Any valid rotation quaternion must have magnitude 1."""
        from tb3_odometry.ekf_node import EKFNode
        for yaw in [0.0, 0.5, math.pi / 2, math.pi, -1.0]:
            q    = EKFNode._yaw_to_quaternion(yaw)
            mag  = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
            assert mag == pytest.approx(1.0, abs=1e-9)


# ── TestPredict ────────────────────────────────────────────────────────────

class TestPredict:
    """Tests for the EKF prediction step."""

    def test_stationary_robot_no_motion(self, node):
        """v=0, w=0 → state unchanged."""
        state, P = fresh_state()
        state_new, P_new = node._predict(state, P, v=0.0, w=0.0, dt=0.1)
        assert state_new[0] == pytest.approx(0.0)
        assert state_new[1] == pytest.approx(0.0)
        assert state_new[2] == pytest.approx(0.0)

    def test_covariance_grows_when_stationary(self, node):
        """Even with zero velocity, covariance grows due to process noise Q."""
        state, P = fresh_state()
        P_initial = P.copy()
        _, P_new = node._predict(state, P, v=0.0, w=0.0, dt=0.1)
        # Diagonal must be larger (we added Q)
        assert P_new[0, 0] > P_initial[0, 0]
        assert P_new[1, 1] > P_initial[1, 1]
        assert P_new[2, 2] > P_initial[2, 2]

    def test_forward_motion_updates_x(self, node):
        """Heading θ=0, v=1.0 m/s, dt=1.0 s → x increases by ~1 m."""
        state = np.array([0.0, 0.0, 0.0])
        _, P  = fresh_state()
        state_new, _ = node._predict(state, P, v=1.0, w=0.0, dt=1.0)
        assert state_new[0] == pytest.approx(1.0, abs=1e-9)
        assert state_new[1] == pytest.approx(0.0, abs=1e-9)

    def test_sideways_motion_at_90_degrees(self, node):
        """Heading θ=π/2 (facing left), v=1.0 → y increases by ~1 m."""
        state = np.array([0.0, 0.0, math.pi / 2])
        _, P  = fresh_state()
        state_new, _ = node._predict(state, P, v=1.0, w=0.0, dt=1.0)
        assert state_new[0] == pytest.approx(0.0, abs=1e-6)
        assert state_new[1] == pytest.approx(1.0, abs=1e-6)

    def test_rotation_updates_heading(self, node):
        """w=1.0 rad/s, dt=0.5 s → theta increases by 0.5 rad."""
        state = np.array([0.0, 0.0, 0.0])
        _, P  = fresh_state()
        state_new, _ = node._predict(state, P, v=0.0, w=1.0, dt=0.5)
        assert state_new[2] == pytest.approx(0.5, abs=1e-9)

    def test_heading_normalised_past_pi(self, node):
        """Heading that wraps above π must be normalised to [-π, π]."""
        state = np.array([0.0, 0.0, math.pi - 0.1])
        _, P  = fresh_state()
        # Angular velocity pushes theta past π
        state_new, _ = node._predict(state, P, v=0.0, w=1.0, dt=0.5)
        assert -math.pi <= state_new[2] <= math.pi

    def test_predict_returns_numpy_arrays(self, node):
        state, P = fresh_state()
        state_new, P_new = node._predict(state, P, v=0.5, w=0.1, dt=0.1)
        assert isinstance(state_new, np.ndarray)
        assert isinstance(P_new, np.ndarray)
        assert state_new.shape == (3,)
        assert P_new.shape == (3, 3)


# ── TestCorrect ────────────────────────────────────────────────────────────

class TestCorrect:
    """Tests for the EKF correction step."""

    def test_correct_pulls_heading_toward_imu(self, node):
        """If θ_imu > θ_pred, heading must increase after correction."""
        state    = np.array([0.0, 0.0, 0.0])   # predicted heading = 0
        P        = np.eye(3) * 0.5
        theta_imu = 0.3                          # IMU says 0.3 rad

        state_new, _ = node._correct(state, P, theta_imu)
        # Heading must move toward 0.3 (between 0 and 0.3)
        assert 0.0 < state_new[2] < 0.3

    def test_correct_does_not_move_when_heading_matches(self, node):
        """If θ_imu == θ_pred, heading should not change."""
        state     = np.array([0.0, 0.0, 0.5])
        P         = np.eye(3) * 0.5
        theta_imu = 0.5

        state_new, _ = node._correct(state, P, theta_imu)
        assert state_new[2] == pytest.approx(0.5, abs=1e-9)

    def test_covariance_shrinks_after_correction(self, node):
        """Getting a measurement must reduce the heading uncertainty."""
        state    = np.array([0.0, 0.0, 0.0])
        P        = np.eye(3) * 0.5
        _, P_new = node._correct(state, P, theta_imu=0.0)
        assert P_new[2, 2] < P[2, 2]

    def test_correct_handles_angle_wraparound(self, node):
        """Correction near ±π boundary must not jump across the discontinuity."""
        # Predicted heading is just below π, IMU says just above -π
        state     = np.array([0.0, 0.0, math.pi - 0.05])
        P         = np.eye(3) * 0.5
        theta_imu = -(math.pi - 0.05)   # same angle, opposite sign

        state_new, _ = node._correct(state, P, theta_imu)
        # Result must be in [-π, π] — no overflow
        assert -math.pi <= state_new[2] <= math.pi

    def test_correct_returns_numpy_arrays(self, node):
        state, P = fresh_state()
        state_new, P_new = node._correct(state, P, theta_imu=0.1)
        assert isinstance(state_new, np.ndarray)
        assert isinstance(P_new, np.ndarray)

    def test_x_y_barely_change_from_heading_correction(self, node):
        """
        A heading correction does not directly move x or y, but K is non-zero
        for x and y (off-diagonal in P). With identity P the x/y correction
        is tiny — it should be MUCH smaller than the heading correction.
        """
        state     = np.array([5.0, 3.0, 0.0])
        P         = np.eye(3) * 0.5
        theta_imu = 1.0

        state_new, _ = node._correct(state, P, theta_imu)
        heading_change = abs(state_new[2] - state[2])
        x_change       = abs(state_new[0] - state[0])
        y_change       = abs(state_new[1] - state[1])
        # Heading correction must dominate over x/y changes
        assert heading_change > x_change
        assert heading_change > y_change


# ── TestPredictCorrectIntegration ──────────────────────────────────────────

class TestPredictCorrectIntegration:
    """End-to-end: run a few predict+correct cycles and check sanity."""

    def test_full_cycle_heading_converges(self, node):
        """
        Robot drives straight (v=0.5, w=0). IMU always reports θ=0.
        After several predict+correct cycles, heading stays near 0.
        """
        state = np.zeros(3)
        P     = np.eye(3) * 0.5

        for _ in range(10):
            state, P = node._predict(state, P, v=0.5, w=0.0, dt=0.1)
            state, P = node._correct(state, P, theta_imu=0.0)

        # After 10 steps of 0.1 s at 0.5 m/s, x ≈ 0.5 m
        assert state[0] == pytest.approx(0.5, abs=0.1)
        # Heading should stay close to 0 (IMU keeps pulling it back)
        assert state[2] == pytest.approx(0.0, abs=0.05)

    def test_full_cycle_covariance_stabilises(self, node):
        """
        After many predict+correct cycles, covariance should neither blow up
        nor collapse to zero.
        """
        state = np.zeros(3)
        P     = np.eye(3) * 1.0

        for _ in range(50):
            state, P = node._predict(state, P, v=0.2, w=0.0, dt=0.1)
            state, P = node._correct(state, P, theta_imu=0.0)

        # Heading covariance should be small but positive
        assert 0.0 < P[2, 2] < 0.1
        # x/y covariance will grow (no position measurement), but not explode
        assert P[0, 0] < 1000.0
