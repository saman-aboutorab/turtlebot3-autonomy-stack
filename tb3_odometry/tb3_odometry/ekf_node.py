#!/usr/bin/env python3
"""
ekf_node.py
===========
Extended Kalman Filter that fuses wheel odometry (/odom) with IMU heading
(/imu/data) to produce a cleaner pose estimate on /odometry/filtered.

WHY THIS NODE EXISTS:
  Wheel odometry alone accumulates unbounded error because:
    1. Small heading errors compound — a 1° heading error means every 1 m you
       travel, your estimated position drifts 1.7 cm sideways. After 10 m it
       is 17 cm off, with no mechanism to correct.
    2. Wheel slip breaks the encoder→distance assumption entirely.

  The IMU provides an INDEPENDENT heading measurement (from the DMP orientation
  filter — not wheel counting). By fusing both sensors with a Kalman filter, we
  get bounded heading uncertainty: every IMU update pulls the estimate back
  toward the true heading.

THE EXTENDED KALMAN FILTER (EKF) — PLAIN ENGLISH:
  A Kalman filter is a two-step loop, run every time the odometry publishes:

    STEP 1 — PREDICT:
      "Based on the wheel velocities in the latest /odom message,
       where do I think the robot is now?"
      Uses the non-linear bicycle motion model:
        x_new = x + v * cos(θ) * dt
        y_new = y + v * sin(θ) * dt
        θ_new = θ + ω * dt
      We also grow the uncertainty (covariance P) because time has passed
      and we know the wheels are not perfect.

    STEP 2 — CORRECT (if IMU data is available):
      "The IMU says the heading is θ_imu.
       My prediction says it is θ_pred.
       How much should I trust each?"
      The Kalman gain K decides the trust ratio:
        K = P * H^T * (H * P * H^T + R)^{-1}
      High P (uncertain prediction) → K is large  → pull hard toward IMU
      High R (uncertain IMU)        → K is small  → keep the prediction

      Then we update the state and shrink the covariance:
        x = x_pred + K * (θ_imu − θ_pred)
        P = (I − K * H) * P

WHY "EXTENDED" KALMAN:
  A standard Kalman filter only works for LINEAR motion models.
  Our model is non-linear (cos θ, sin θ).
  We linearise it at each step using the Jacobian F = d(model)/d(state):
    F = [[1, 0, −v * sin(θ) * dt],
         [0, 1,  v * cos(θ) * dt],
         [0, 0,  1              ]]
  This is the "Extended" part — we recompute F every prediction step.

STATE VECTOR (3-DOF):
  x = [x_pos, y_pos, theta]   (metres, metres, radians)

  We do NOT track velocities in the state — they come directly from the
  odometry message as "control inputs" (v, ω). This keeps the filter simple.

MEASUREMENT MODEL:
  z = θ_imu   (yaw extracted from IMU quaternion via the formula:
                yaw = atan2(2*(w*z + x*y), 1 − 2*(y² + z²)))
  H = [[0, 0, 1]]  — we observe only the heading component of the state
  R = [[measurement_noise_imu]]  — 1×1 matrix

COVARIANCE MATRICES:
  P  (3×3): state uncertainty — starts large, shrinks as we get measurements
  Q  (3×3): process noise — how much we distrust the motion model per step
  R  (1×1): measurement noise — how much we distrust the IMU heading

  Tuning rule of thumb:
    Q_pos large     → predict conservatively, trust IMU more
    Q_heading large → let heading drift, rely on IMU correction
    R large         → IMU is noisy, trust prediction more

PARAMETERS:
  process_noise_pos     (float) default: 0.01    m²  per second
  process_noise_heading (float) default: 0.001   rad² per second
  measurement_noise_imu (float) default: 0.0001  rad² (from imu_republisher)
  odom_frame_id         (str)   default: 'odom'
  base_frame_id         (str)   default: 'base_footprint'

SUBSCRIPTIONS:
  /odom        nav_msgs/Odometry   — wheel odometry (prediction trigger)
  /imu/data    sensor_msgs/Imu     — cleaned IMU from imu_republisher.py

PUBLICATIONS:
  /odometry/filtered   nav_msgs/Odometry   — EKF-fused pose estimate
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class EKFNode(Node):

    def __init__(self):
        super().__init__('ekf_node')

        # ----------------------------------------------------------------
        # PARAMETERS
        # ----------------------------------------------------------------
        self.declare_parameter('process_noise_pos',     0.01)
        self.declare_parameter('process_noise_heading', 0.001)
        self.declare_parameter('measurement_noise_imu', 0.0001)
        self.declare_parameter('odom_frame_id',         'odom')
        self.declare_parameter('base_frame_id',         'base_footprint')

        proc_pos     = self.get_parameter('process_noise_pos').value
        proc_heading = self.get_parameter('process_noise_heading').value
        meas_imu     = self.get_parameter('measurement_noise_imu').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        # ----------------------------------------------------------------
        # EKF STATE
        # state: numpy array [x, y, theta]
        # P:     3×3 covariance matrix — starts large (we don't know pose)
        # ----------------------------------------------------------------
        self._state = np.zeros(3)
        self._P     = np.eye(3) * 1.0   # 1 m² / 1 rad² initial uncertainty

        # ----------------------------------------------------------------
        # NOISE MATRICES (constant, built once at startup)
        # ----------------------------------------------------------------
        # Process noise Q: grows P every prediction step
        # Diagonal: [x uncertainty, y uncertainty, heading uncertainty]
        self._Q = np.diag([proc_pos, proc_pos, proc_heading])

        # Measurement noise R: 1×1 (we only measure heading from IMU)
        self._R = np.array([[meas_imu]])

        # Measurement matrix H: "we observe element 2 of the state (theta)"
        # H is (1×3): maps 3-DOF state → scalar heading observation
        self._H = np.array([[0.0, 0.0, 1.0]])

        # ----------------------------------------------------------------
        # RUNTIME STATE
        # ----------------------------------------------------------------
        self._last_odom_time = None     # float seconds, for computing dt
        self._imu_heading    = None     # latest θ from IMU (float radians)
        self._imu_available  = False    # True once first /imu/data arrives

        # ----------------------------------------------------------------
        # SUBSCRIBERS
        # Both topics use BEST_EFFORT to match their publishers.
        # ----------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, sensor_qos
        )
        self._imu_sub = self.create_subscription(
            Imu, '/imu/data', self._imu_callback, sensor_qos
        )

        # ----------------------------------------------------------------
        # PUBLISHER — /odometry/filtered
        # Uses RELIABLE QoS: downstream nodes (tf2_broadcaster, Nav2) need
        # every message. No sensor-style dropping here.
        # ----------------------------------------------------------------
        self._filtered_pub = self.create_publisher(
            Odometry, '/odometry/filtered', 10
        )

        self.get_logger().info(
            f'EKFNode started.\n'
            f'  process_noise_pos:     {proc_pos}\n'
            f'  process_noise_heading: {proc_heading}\n'
            f'  measurement_noise_imu: {meas_imu}\n'
            f'  subscribing to:        /odom, /imu/data\n'
            f'  publishing to:         /odometry/filtered'
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def _imu_callback(self, msg: Imu) -> None:
        """
        Store the latest IMU heading so the prediction step can use it.

        We extract yaw from the quaternion using the standard formula.
        The DMP filter inside the OpenCR has already fused gyro + accel
        to produce this orientation — we just read the result.

        We do NOT run the EKF correction here. The correction is triggered
        by the odometry callback so that the EKF output rate matches the
        odometry rate (not the faster IMU rate).
        """
        self._imu_heading   = self._yaw_from_quaternion(msg.orientation)
        self._imu_available = True

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Main EKF loop: called every time a new /odom message arrives.

        1. Compute dt since last message.
        2. Run the PREDICT step (motion model + covariance growth).
        3. Run the CORRECT step (IMU heading update, if available).
        4. Publish the updated state on /odometry/filtered.
        """
        # --- Compute dt ---
        stamp = msg.header.stamp
        t     = stamp.sec + stamp.nanosec * 1e-9

        if self._last_odom_time is None:
            # First message: initialise timestamp only, no prediction yet.
            self._last_odom_time = t
            return

        dt = t - self._last_odom_time
        self._last_odom_time = t

        if dt <= 0.0 or dt > 1.0:
            # Guard against bad timestamps or long gaps (e.g., after pause).
            # A gap > 1 s would make the prediction step unreliable.
            return

        # --- Control inputs from /odom twist ---
        # odometry_publisher.py fills in the velocity at the time of the
        # pose estimate, which is exactly what the prediction step needs.
        v = msg.twist.twist.linear.x    # linear velocity  (m/s)
        w = msg.twist.twist.angular.z   # angular velocity (rad/s)

        # --- PREDICT ---
        self._state, self._P = self._predict(self._state, self._P, v, w, dt)

        # --- CORRECT (only once we have at least one IMU reading) ---
        if self._imu_available and self._imu_heading is not None:
            self._state, self._P = self._correct(
                self._state, self._P, self._imu_heading
            )

        # --- Publish ---
        self._publish_filtered(stamp)

    # ================================================================
    # EKF CORE METHODS (pure functions — no ROS2 I/O, fully testable)
    # ================================================================

    def _predict(
        self,
        state: np.ndarray,
        P: np.ndarray,
        v: float,
        w: float,
        dt: float
    ):
        """
        EKF prediction step.

        Applies the non-linear motion model to advance the state, then
        propagates the covariance through the linearised Jacobian.

        Args:
            state: current [x, y, theta]
            P:     current 3×3 covariance
            v:     linear  velocity (m/s) from odometry
            w:     angular velocity (rad/s) from odometry
            dt:    time since last prediction (seconds)

        Returns:
            (state_new, P_new): predicted state and covariance
        """
        x, y, theta = state

        # Non-linear motion model (same as odometry_publisher's kinematics)
        x_new     = x + v * math.cos(theta) * dt
        y_new     = y + v * math.sin(theta) * dt
        theta_new = theta + w * dt

        # Keep heading in [-π, π]
        theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))

        state_new = np.array([x_new, y_new, theta_new])

        # Jacobian F = d(model)/d(state), evaluated at current theta.
        # This linearises the motion model so we can propagate the covariance.
        # Row 0: d(x_new)/d(x, y, θ)  →  [1, 0, -v·sin(θ)·dt]
        # Row 1: d(y_new)/d(x, y, θ)  →  [0, 1,  v·cos(θ)·dt]
        # Row 2: d(θ_new)/d(x, y, θ)  →  [0, 0,  1           ]
        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0,  1.0                      ],
        ])

        # Covariance propagation:  P = F * P * F^T + Q
        # F*P*F^T: how prediction uncertainty maps through the Jacobian
        # + Q:     adds process noise (distrust of the motion model)
        P_new = F @ P @ F.T + self._Q

        return state_new, P_new

    def _correct(
        self,
        state: np.ndarray,
        P: np.ndarray,
        theta_imu: float
    ):
        """
        EKF correction step.

        Uses the IMU heading as a measurement to pull the state estimate
        toward the true heading. The Kalman gain decides how much to pull.

        Args:
            state:     predicted [x, y, theta] from _predict()
            P:         predicted 3×3 covariance from _predict()
            theta_imu: measured heading from IMU (radians)

        Returns:
            (state_new, P_new): corrected state and covariance
        """
        H = self._H   # (1×3): measurement matrix
        R = self._R   # (1×1): measurement noise

        # Innovation: how far is the IMU reading from our prediction?
        # We wrap the difference to [-π, π] to handle the ±π discontinuity.
        theta_pred = state[2]
        innovation = math.atan2(
            math.sin(theta_imu - theta_pred),
            math.cos(theta_imu - theta_pred)
        )

        # Innovation covariance S = H * P * H^T + R  (scalar, 1×1)
        # S tells us: how uncertain is our heading prediction + measurement?
        S = H @ P @ H.T + R

        # Kalman gain K = P * H^T * S^{-1}   (3×1 vector)
        # K answers: "for each unit of innovation, how much do we adjust
        # x, y, and theta?"
        K = P @ H.T @ np.linalg.inv(S)     # shape (3, 1)

        # State update: pull the predicted state toward the measurement
        # K.flatten() is (3,), innovation is scalar → result is (3,)
        state_new    = state + K.flatten() * innovation
        state_new[2] = math.atan2(
            math.sin(state_new[2]), math.cos(state_new[2])
        )

        # Covariance update: uncertainty shrinks after we get a measurement
        # Standard form: P = (I - K * H) * P
        I     = np.eye(3)
        P_new = (I - K @ H) @ P

        return state_new, P_new

    # ================================================================
    # PUBLISHER HELPER
    # ================================================================

    def _publish_filtered(self, stamp) -> None:
        """Build and publish nav_msgs/Odometry on /odometry/filtered."""
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.odom_frame_id    # 'odom'
        msg.child_frame_id  = self.base_frame_id    # 'base_footprint'

        x, y, theta = self._state

        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation   = self._yaw_to_quaternion(theta)

        # Map EKF covariance P (3×3) into the 6×6 pose covariance matrix.
        # nav_msgs/Odometry uses [x, y, z, roll, pitch, yaw] order.
        # We only track [x, y, yaw], so indices are: x→0, y→1, yaw→5.
        # All other entries (z, roll, pitch) stay 0.
        cov        = [0.0] * 36
        cov[0]     = float(self._P[0, 0])   # x-x
        cov[1]     = float(self._P[0, 1])   # x-y
        cov[5]     = float(self._P[0, 2])   # x-yaw
        cov[6]     = float(self._P[1, 0])   # y-x
        cov[7]     = float(self._P[1, 1])   # y-y
        cov[11]    = float(self._P[1, 2])   # y-yaw
        cov[30]    = float(self._P[2, 0])   # yaw-x
        cov[31]    = float(self._P[2, 1])   # yaw-y
        cov[35]    = float(self._P[2, 2])   # yaw-yaw

        msg.pose.covariance = cov

        self._filtered_pub.publish(msg)

    # ================================================================
    # STATIC HELPERS (pure functions — no node state)
    # ================================================================

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        """
        Extract yaw (rotation around Z axis) from a geometry_msgs/Quaternion.

        Uses the standard Euler-angle formula:
          yaw = atan2(2*(w*z + x*y),  1 - 2*(y² + z²))

        This is the inverse of _yaw_to_quaternion.
        Returns a value in [-π, π].
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """
        Convert a yaw angle (rotation around Z) to a geometry_msgs/Quaternion.

        For a pure yaw rotation:
          qx = 0, qy = 0, qz = sin(yaw/2), qw = cos(yaw/2)
        """
        q   = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
