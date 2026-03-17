"""
test_imu_republisher.py
=======================
Unit tests for ImuRepublisher.

Tests focus on _clean_imu() — the only logic in this node.
Verifies that:
  - sensor data passes through unchanged
  - frame_id is overridden to 'imu_link'
  - timestamp is preserved from raw message
  - covariances are set to non-zero diagonal values
  - covariance[0] is never -1 (which would disable the sensor in EKF)

Run with:
  python3 -m pytest tb3_odometry/test/test_imu_republisher.py -v
"""

import math
import pytest

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu


def make_raw_imu(sec=1, orient=(0.0, 0.0, 0.0, 1.0),
                 gyro=(0.1, 0.2, 0.3),
                 accel=(0.0, 0.0, 9.81)) -> Imu:
    """Build a raw Imu message with zero covariances (like OpenCR output)."""
    msg = Imu()
    msg.header.stamp    = Time(sec=sec, nanosec=0)
    msg.header.frame_id = ''          # empty — the bug we're fixing

    msg.orientation = Quaternion(
        x=orient[0], y=orient[1], z=orient[2], w=orient[3]
    )
    msg.angular_velocity    = Vector3(x=gyro[0],  y=gyro[1],  z=gyro[2])
    msg.linear_acceleration = Vector3(x=accel[0], y=accel[1], z=accel[2])

    # All-zero covariances — the problem we're fixing
    msg.orientation_covariance          = [0.0] * 9
    msg.angular_velocity_covariance     = [0.0] * 9
    msg.linear_acceleration_covariance  = [0.0] * 9

    return msg


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_init):
    from tb3_odometry.imu_republisher import ImuRepublisher
    n = ImuRepublisher()
    yield n
    n.destroy_node()


# ── tests ──────────────────────────────────────────────────────────────────

class TestFrameId:

    def test_frame_id_set_to_imu_link(self, node):
        """frame_id must be 'imu_link' regardless of what the raw message had."""
        raw = make_raw_imu()
        clean = node._clean_imu(raw)
        assert clean.header.frame_id == 'imu_link'

    def test_raw_empty_frame_id_is_replaced(self, node):
        """OpenCR sends empty frame_id — must be replaced, not kept."""
        raw = make_raw_imu()
        raw.header.frame_id = ''
        clean = node._clean_imu(raw)
        assert clean.header.frame_id != ''
        assert clean.header.frame_id == 'imu_link'


class TestTimestamp:

    def test_timestamp_preserved_from_raw(self, node):
        """Timestamp comes from the raw message, not the system clock."""
        raw = make_raw_imu(sec=99)
        clean = node._clean_imu(raw)
        assert clean.header.stamp.sec == 99


class TestSensorDataPassthrough:
    """Sensor readings must pass through completely unchanged."""

    def test_orientation_unchanged(self, node):
        raw = make_raw_imu(orient=(0.1, 0.2, 0.3, 0.9))
        clean = node._clean_imu(raw)
        assert clean.orientation.x == pytest.approx(0.1)
        assert clean.orientation.y == pytest.approx(0.2)
        assert clean.orientation.z == pytest.approx(0.3)
        assert clean.orientation.w == pytest.approx(0.9)

    def test_angular_velocity_unchanged(self, node):
        raw = make_raw_imu(gyro=(0.5, -0.3, 0.1))
        clean = node._clean_imu(raw)
        assert clean.angular_velocity.x == pytest.approx(0.5)
        assert clean.angular_velocity.y == pytest.approx(-0.3)
        assert clean.angular_velocity.z == pytest.approx(0.1)

    def test_linear_acceleration_unchanged(self, node):
        raw = make_raw_imu(accel=(0.02, -0.01, 9.81))
        clean = node._clean_imu(raw)
        assert clean.linear_acceleration.x == pytest.approx(0.02)
        assert clean.linear_acceleration.y == pytest.approx(-0.01)
        assert clean.linear_acceleration.z == pytest.approx(9.81)


class TestCovariances:

    def test_orientation_covariance_diagonal_nonzero(self, node):
        """Diagonal of orientation covariance must be non-zero (EKF requirement)."""
        raw = make_raw_imu()
        clean = node._clean_imu(raw)
        cov = clean.orientation_covariance
        assert cov[0] > 0.0   # xx
        assert cov[4] > 0.0   # yy
        assert cov[8] > 0.0   # zz

    def test_angular_velocity_covariance_diagonal_nonzero(self, node):
        raw = make_raw_imu()
        clean = node._clean_imu(raw)
        cov = clean.angular_velocity_covariance
        assert cov[0] > 0.0
        assert cov[4] > 0.0
        assert cov[8] > 0.0

    def test_linear_accel_covariance_diagonal_nonzero(self, node):
        raw = make_raw_imu()
        clean = node._clean_imu(raw)
        cov = clean.linear_acceleration_covariance
        assert cov[0] > 0.0
        assert cov[4] > 0.0
        assert cov[8] > 0.0

    def test_no_covariance_minus_one(self, node):
        """covariance[0] = -1 means 'ignore this sensor' in EKF. Must never happen."""
        raw = make_raw_imu()
        clean = node._clean_imu(raw)
        assert clean.orientation_covariance[0]         != -1.0
        assert clean.angular_velocity_covariance[0]    != -1.0
        assert clean.linear_acceleration_covariance[0] != -1.0

    def test_off_diagonal_terms_are_zero(self, node):
        """Axes assumed independent — off-diagonal covariance terms must be 0."""
        raw = make_raw_imu()
        clean = node._clean_imu(raw)
        cov = clean.orientation_covariance
        off_diag = [cov[1], cov[2], cov[3], cov[5], cov[6], cov[7]]
        for val in off_diag:
            assert val == 0.0

    def test_covariance_not_shared_between_messages(self, node):
        """Each message must have its own covariance list (no aliasing)."""
        raw = make_raw_imu()
        clean_a = node._clean_imu(raw)
        clean_b = node._clean_imu(raw)
        clean_a.orientation_covariance[0] = 999.0
        assert clean_b.orientation_covariance[0] != 999.0


class TestDiagonalCovariance:
    """Tests for the static _diagonal_covariance helper."""

    def test_returns_nine_values(self):
        from tb3_odometry.imu_republisher import ImuRepublisher
        result = ImuRepublisher._diagonal_covariance(0.05)
        assert len(result) == 9

    def test_diagonal_values_set(self):
        from tb3_odometry.imu_republisher import ImuRepublisher
        result = ImuRepublisher._diagonal_covariance(0.05)
        assert result[0] == pytest.approx(0.05)
        assert result[4] == pytest.approx(0.05)
        assert result[8] == pytest.approx(0.05)

    def test_off_diagonal_zero(self):
        from tb3_odometry.imu_republisher import ImuRepublisher
        result = ImuRepublisher._diagonal_covariance(0.05)
        assert result[1] == 0.0
        assert result[2] == 0.0
        assert result[3] == 0.0
