"""
test_velocity_controller.py
===========================

Unit tests for velocity_controller.py.

Tests are organised into two groups:

  1. Pure logic tests (_clamp, _ramp) — no ROS2 needed, run instantly.
  2. Node parameter tests — spin up the node once, check default values.

All the interesting behaviour lives in the pure helper methods, so that is
where most of the test coverage is focused.
"""

import pytest


# ── rclpy fixture (shared for all tests in this file) ────────────────────────

@pytest.fixture(scope='module')
def rclpy_init():
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='module')
def node(rclpy_init):
    from tb3_odometry.velocity_controller import VelocityController
    n = VelocityController()
    yield n
    n.destroy_node()


# ── _clamp tests ─────────────────────────────────────────────────────────────

class TestClamp:
    """_clamp(value, lo, hi) must constrain value to [lo, hi]."""

    def test_within_range_unchanged(self):
        from tb3_odometry.velocity_controller import VelocityController
        assert VelocityController._clamp(0.1, -0.22, 0.22) == pytest.approx(0.1)

    def test_above_max_clamped(self):
        from tb3_odometry.velocity_controller import VelocityController
        assert VelocityController._clamp(0.5, -0.22, 0.22) == pytest.approx(0.22)

    def test_below_min_clamped(self):
        from tb3_odometry.velocity_controller import VelocityController
        assert VelocityController._clamp(-0.5, -0.22, 0.22) == pytest.approx(-0.22)

    def test_at_max_boundary(self):
        from tb3_odometry.velocity_controller import VelocityController
        assert VelocityController._clamp(0.22, -0.22, 0.22) == pytest.approx(0.22)

    def test_at_min_boundary(self):
        from tb3_odometry.velocity_controller import VelocityController
        assert VelocityController._clamp(-0.22, -0.22, 0.22) == pytest.approx(-0.22)

    def test_zero_unchanged(self):
        from tb3_odometry.velocity_controller import VelocityController
        assert VelocityController._clamp(0.0, -0.22, 0.22) == pytest.approx(0.0)

    def test_angular_above_max(self):
        from tb3_odometry.velocity_controller import VelocityController
        assert VelocityController._clamp(5.0, -2.84, 2.84) == pytest.approx(2.84)


# ── _ramp tests ───────────────────────────────────────────────────────────────

class TestRamp:
    """
    _ramp(current, target, max_acc, dt) advances current toward target
    by at most max_acc * dt per call.
    """

    def test_small_delta_reaches_target_in_one_step(self):
        """If the gap is smaller than max_step, go all the way."""
        from tb3_odometry.velocity_controller import VelocityController
        # max_step = 0.5 * 0.05 = 0.025 ; delta = 0.01 < 0.025
        result = VelocityController._ramp(0.0, 0.01, 0.5, 0.05)
        assert result == pytest.approx(0.01)

    def test_large_delta_limited_to_max_step(self):
        """If the gap is larger than max_step, advance only max_step."""
        from tb3_odometry.velocity_controller import VelocityController
        # max_step = 0.5 * 0.05 = 0.025 ; delta = 0.22 > 0.025
        result = VelocityController._ramp(0.0, 0.22, 0.5, 0.05)
        assert result == pytest.approx(0.025)

    def test_deceleration_is_also_limited(self):
        """Ramping down is symmetric with ramping up."""
        from tb3_odometry.velocity_controller import VelocityController
        # current = 0.22, target = 0.0, max_step = 0.025
        result = VelocityController._ramp(0.22, 0.0, 0.5, 0.05)
        assert result == pytest.approx(0.22 - 0.025)

    def test_negative_target_ramps_correctly(self):
        """Accelerating into reverse is limited too."""
        from tb3_odometry.velocity_controller import VelocityController
        # max_step = 0.025 ; delta = -0.22 → step = -0.025
        result = VelocityController._ramp(0.0, -0.22, 0.5, 0.05)
        assert result == pytest.approx(-0.025)

    def test_already_at_target_no_movement(self):
        """No change when current == target."""
        from tb3_odometry.velocity_controller import VelocityController
        result = VelocityController._ramp(0.1, 0.1, 0.5, 0.05)
        assert result == pytest.approx(0.1)

    def test_zero_to_zero(self):
        from tb3_odometry.velocity_controller import VelocityController
        assert VelocityController._ramp(0.0, 0.0, 0.5, 0.05) == pytest.approx(0.0)

    def test_full_ramp_reaches_target(self):
        """After enough steps the ramp converges to the target."""
        from tb3_odometry.velocity_controller import VelocityController
        # max_step = 0.5 * 0.05 = 0.025 per step
        # 0.22 / 0.025 = 8.8 steps → should reach target by step 9
        current = 0.0
        target  = 0.22
        for _ in range(20):
            current = VelocityController._ramp(current, target, 0.5, 0.05)
        assert current == pytest.approx(target, abs=1e-9)

    def test_ramp_never_overshoots(self):
        """Ramp must not go past the target."""
        from tb3_odometry.velocity_controller import VelocityController
        current = 0.0
        target  = 0.22
        for _ in range(50):
            current = VelocityController._ramp(current, target, 0.5, 0.05)
            assert current <= target + 1e-9

    def test_ramp_never_undershoots_on_decel(self):
        """Ramp must not drop below target while decelerating."""
        from tb3_odometry.velocity_controller import VelocityController
        current = 0.22
        target  = 0.0
        for _ in range(50):
            current = VelocityController._ramp(current, target, 0.5, 0.05)
            assert current >= target - 1e-9


# ── node default parameter tests ─────────────────────────────────────────────

class TestNodeDefaults:
    """Verify default parameter values match TurtleBot3 Burger specs."""

    def test_default_max_linear_vel(self, node):
        assert node._max_lv == pytest.approx(0.22)

    def test_default_max_angular_vel(self, node):
        assert node._max_av == pytest.approx(2.84)

    def test_default_max_linear_acc(self, node):
        assert node._max_la == pytest.approx(0.50)

    def test_default_max_angular_acc(self, node):
        assert node._max_aa == pytest.approx(3.00)

    def test_default_timeout(self, node):
        assert node._timeout == pytest.approx(0.5)

    def test_initial_target_velocities_zero(self, node):
        assert node._target_lv == pytest.approx(0.0)
        assert node._target_av == pytest.approx(0.0)

    def test_initial_current_velocities_zero(self, node):
        assert node._current_lv == pytest.approx(0.0)
        assert node._current_av == pytest.approx(0.0)
