#!/usr/bin/env python3
"""
velocity_controller.py
======================

Sits between the motion planner (Nav2 / teleop) and the robot's drive hardware.
Applies three safety layers before forwarding a velocity command to the wheels:

  1. VELOCITY CLAMPING    — hard upper/lower limits on linear and angular speed
  2. ACCELERATION LIMITING — ramp: speed cannot jump instantaneously
  3. SAFETY TIMEOUT        — if no new command arrives within timeout_secs, stop

WHY THIS EXISTS:
  Nav2 and teleop nodes issue /cmd_vel commands without knowing the physical
  limits of the specific robot. Without a controller in between:

    - A /cmd_vel with linear.x = 2.0 m/s would be sent to a Burger that tops
      out at 0.22 m/s. The OpenCR clamps it internally, but in a discontinuous
      way that causes wheel slip and odometry jumps.

    - A sudden jump from 0 to 0.22 m/s in one step can cause wheel slip that
      corrupts the encoder-based odometry.

    - If the planner crashes mid-motion, the robot will keep moving at the last
      commanded velocity until something stops it. The timeout prevents this.

  This node addresses all three issues in a single, testable Python module.

HOW IT CONNECTS:
  ┌──────────────────┐      /cmd_vel_raw      ┌─────────────────────┐
  │  Nav2 / teleop   │  ─────────────────────► │ velocity_controller │
  └──────────────────┘                         │                     │
                                               │  1. clamp           │
                                               │  2. ramp            │  /cmd_vel
                                               │  3. timeout check   │ ──────────► OpenCR
                                               └─────────────────────┘

DESIGN PATTERN — timer-driven output:
  The subscriber (_cmd_callback) only stores the target velocity; it does NOT
  publish anything directly. A fixed-rate timer (_control_loop) reads the target
  and applies ramping before publishing. This decouples the input rate (variable,
  from Nav2/teleop) from the output rate (fixed, configurable via control_rate).
  It also means the timeout check fires reliably even if /cmd_vel_raw goes silent.

PURE HELPER METHODS:
  _clamp() and _ramp() contain the core logic and have no ROS2 dependencies.
  This makes them directly unit-testable without a running ROS2 node.

PARAMETERS:
  max_linear_vel   (float) default: 0.22   m/s    TurtleBot3 Burger physical limit
  max_angular_vel  (float) default: 2.84   rad/s  TurtleBot3 Burger physical limit
  max_linear_acc   (float) default: 0.50   m/s²   ramp rate for linear velocity
  max_angular_acc  (float) default: 3.00   rad/s² ramp rate for angular velocity
  control_rate     (float) default: 20.0   Hz     output publish rate
  timeout_secs     (float) default: 0.5    s      stop if no input for this long
  input_topic      (str)   default: '/cmd_vel_raw'
  output_topic     (str)   default: '/cmd_vel'
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityController(Node):
    def __init__(self) -> None:
        super().__init__('velocity_controller')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('max_linear_vel',  0.22)
        self.declare_parameter('max_angular_vel', 2.84)
        self.declare_parameter('max_linear_acc',  0.50)
        self.declare_parameter('max_angular_acc', 3.00)
        self.declare_parameter('control_rate',    20.0)
        self.declare_parameter('timeout_secs',    0.5)
        self.declare_parameter('input_topic',     '/cmd_vel_raw')
        self.declare_parameter('output_topic',    '/cmd_vel')

        self._max_lv  = self.get_parameter('max_linear_vel').value
        self._max_av  = self.get_parameter('max_angular_vel').value
        self._max_la  = self.get_parameter('max_linear_acc').value
        self._max_aa  = self.get_parameter('max_angular_acc').value
        rate          = self.get_parameter('control_rate').value
        self._timeout = self.get_parameter('timeout_secs').value
        in_topic      = self.get_parameter('input_topic').value
        out_topic     = self.get_parameter('output_topic').value

        # ── state ────────────────────────────────────────────────────────────
        # _target:  what the latest /cmd_vel_raw requested (already clamped)
        # _current: what we are actually outputting right now (ramped toward target)
        self._target_lv  = 0.0
        self._target_av  = 0.0
        self._current_lv = 0.0
        self._current_av = 0.0

        # time of the last received command — used by the timeout check
        self._last_cmd_time = self.get_clock().now()

        # ── ROS2 interfaces ──────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Twist, in_topic, self._cmd_callback, 10
        )
        self._pub = self.create_publisher(Twist, out_topic, 10)

        # The control loop fires at a fixed rate regardless of input frequency.
        self._dt    = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'VelocityController started.\n'
            f'  max_linear_vel:  {self._max_lv} m/s\n'
            f'  max_angular_vel: {self._max_av} rad/s\n'
            f'  max_linear_acc:  {self._max_la} m/s²\n'
            f'  max_angular_acc: {self._max_aa} rad/s²\n'
            f'  control_rate:    {rate} Hz  (dt = {self._dt:.4f} s)\n'
            f'  timeout_secs:    {self._timeout} s\n'
            f'  listening:       {in_topic}\n'
            f'  publishing:      {out_topic}'
        )

    # ── subscriber callback ──────────────────────────────────────────────────

    def _cmd_callback(self, msg: Twist) -> None:
        """
        Store the latest velocity request from the planner.

        We only clamp here (hard safety limit). Ramping happens in _control_loop
        so it is tied to the fixed-rate timer, not to the variable input rate.
        """
        self._target_lv     = self._clamp(msg.linear.x,  -self._max_lv, self._max_lv)
        self._target_av     = self._clamp(msg.angular.z, -self._max_av, self._max_av)
        self._last_cmd_time = self.get_clock().now()

    # ── timer callback (main control loop) ──────────────────────────────────

    def _control_loop(self) -> None:
        """
        Called at control_rate Hz.

        1. Check for timeout → zero the target if stale.
        2. Ramp current velocity toward target.
        3. Publish the ramped velocity.
        """
        # --- timeout check ---
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self._timeout:
            # No command received recently → ramp toward a full stop.
            self._target_lv = 0.0
            self._target_av = 0.0

        # --- acceleration limiting (ramp) ---
        self._current_lv = self._ramp(
            self._current_lv, self._target_lv, self._max_la, self._dt
        )
        self._current_av = self._ramp(
            self._current_av, self._target_av, self._max_aa, self._dt
        )

        # --- publish ---
        msg = Twist()
        msg.linear.x  = self._current_lv
        msg.angular.z = self._current_av
        self._pub.publish(msg)

    # ── pure helper methods (no ROS2 — directly unit-testable) ──────────────

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        """
        Clamp value into [lo, hi].

        Example:
          _clamp(0.5, -0.22, 0.22) → 0.22   (exceeded max)
          _clamp(0.1, -0.22, 0.22) → 0.1    (within range, unchanged)
        """
        return max(lo, min(hi, value))

    @staticmethod
    def _ramp(current: float, target: float, max_acc: float, dt: float) -> float:
        """
        Move current toward target, but advance no faster than max_acc * dt
        per time step.

        This is a simple first-order rate limiter. It prevents velocity
        discontinuities (step changes) from reaching the wheels.

        Example (linear acceleration ramp):
          current = 0.0, target = 0.22, max_acc = 0.5, dt = 0.05
          max_step = 0.5 * 0.05 = 0.025
          delta    = 0.22 − 0.0 = 0.22  (> max_step, so we cap it)
          new      = 0.0 + 0.025 = 0.025

          After 9 more identical steps (10 total × 0.05 s = 0.5 s):
          new = 0.25  — now slightly over target, so the next step lands at 0.22
          (The ramp reaches full speed in ≈ 0.22 / 0.5 = 0.44 s)
        """
        max_step = max_acc * dt
        delta    = target - current
        step     = max(-max_step, min(max_step, delta))
        return current + step


# ── entry point ──────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send an explicit stop before shutting down.
        # (The timer will have already sent a zero if the timeout fired, but
        # a final explicit stop is good practice in case shutdown is fast.)
        stop = Twist()
        node._pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
