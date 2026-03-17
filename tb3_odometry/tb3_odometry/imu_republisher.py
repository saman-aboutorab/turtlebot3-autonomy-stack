#!/usr/bin/env python3
"""
imu_republisher.py
==================
Subscribes to /imu (raw sensor_msgs/Imu from the OpenCR board),
fills in correct covariance matrices and frame_id, and republishes
on /imu/data for consumption by the EKF (ekf_node.py, Step 6).

WHY THIS NODE EXISTS:
  The robot_localization EKF requires two things from every IMU message:
    1. header.frame_id must match a valid TF2 frame (our URDF's 'imu_link')
    2. Covariance matrices must have non-zero diagonal values that reflect
       the sensor's actual uncertainty.

  The OpenCR firmware publishes /imu with empty frame_id and zero
  covariances. The EKF would either crash or silently ignore the IMU.
  This node fixes both issues before the data reaches the EKF.

WHAT THE MPU-9250 MEASURES:
  - Gyroscope:     angular velocity (rad/s) around x, y, z axes
  - Accelerometer: linear acceleration (m/s²) including gravity
  - DMP filter:    fused orientation estimate (quaternion)

THE sensor_msgs/Imu MESSAGE:
  header
    frame_id:                 which coordinate frame the data is in
  orientation:                quaternion (from DMP fusion)
  orientation_covariance:     3x3 matrix, uncertainty in orientation
  angular_velocity:           gyro reading (rad/s)
  angular_velocity_covariance: 3x3 matrix, uncertainty in gyro
  linear_acceleration:        accelerometer reading (m/s²)
  linear_acceleration_covariance: 3x3 matrix, uncertainty in accel

  All covariance matrices are 3x3, flattened to 9 values, row-major.
  Order: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
  We only set the diagonal (xx, yy, zz) — off-diagonal terms are
  correlations between axes, which we assume are independent (= 0).

  Special value: covariance[0] = -1 means "this component is unknown,
  ignore it". We must NOT set this — it would disable the sensor.

COVARIANCE VALUES (MPU-9250 datasheet + typical ROS2 tuning):
  Orientation:          ~0.0001 rad² diagonal  (DMP is fairly accurate)
  Angular velocity:     ~0.0001 rad²/s²        (gyro noise spec)
  Linear acceleration:  ~0.001  m²/s⁴          (accel noise spec)

  These are starting values. They can be tuned by measuring real noise
  (record a static robot and compute variance of each axis).

PARAMETERS:
  imu_frame_id              (str)   default: 'imu_link'
  orientation_variance      (float) default: 0.0001
  angular_velocity_variance (float) default: 0.0001
  linear_accel_variance     (float) default: 0.001

SUBSCRIPTIONS:
  /imu           sensor_msgs/Imu   raw IMU from OpenCR

PUBLICATIONS:
  /imu/data      sensor_msgs/Imu   cleaned IMU for EKF
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Imu


class ImuRepublisher(Node):

    def __init__(self):
        super().__init__('imu_republisher')

        # ----------------------------------------------------------------
        # PARAMETERS
        # ----------------------------------------------------------------
        self.declare_parameter('imu_frame_id',              'imu_link')
        self.declare_parameter('orientation_variance',       0.0001)
        self.declare_parameter('angular_velocity_variance',  0.0001)
        self.declare_parameter('linear_accel_variance',      0.001)

        self.imu_frame_id  = self.get_parameter('imu_frame_id').value
        orient_var         = self.get_parameter('orientation_variance').value
        angular_var        = self.get_parameter('angular_velocity_variance').value
        accel_var          = self.get_parameter('linear_accel_variance').value

        # ----------------------------------------------------------------
        # PRE-BUILD the covariance arrays once at startup.
        # A 3x3 diagonal matrix flattened to 9 values:
        #   [var,  0,   0,
        #    0,   var,  0,
        #    0,   0,   var]
        # We build them here so the callback doesn't allocate lists at 50Hz.
        # ----------------------------------------------------------------
        self.orientation_cov = self._diagonal_covariance(orient_var)
        self.angular_vel_cov = self._diagonal_covariance(angular_var)
        self.linear_acc_cov  = self._diagonal_covariance(accel_var)

        # ----------------------------------------------------------------
        # SUBSCRIBER — /imu
        # OpenCR uses BEST_EFFORT QoS for sensor topics.
        # ----------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            sensor_qos
        )

        # ----------------------------------------------------------------
        # PUBLISHER — /imu/data
        # Also BEST_EFFORT — the EKF can tolerate occasional missed messages.
        # ----------------------------------------------------------------
        self.imu_pub = self.create_publisher(Imu, '/imu/data', sensor_qos)

        self.get_logger().info(
            f'ImuRepublisher started.\n'
            f'  frame_id:             {self.imu_frame_id}\n'
            f'  orientation_var:      {orient_var}\n'
            f'  angular_velocity_var: {angular_var}\n'
            f'  linear_accel_var:     {accel_var}\n'
            f'  subscribing to:       /imu\n'
            f'  publishing to:        /imu/data'
        )

    def imu_callback(self, msg: Imu):
        """
        Receives raw IMU message, fixes metadata, republishes.
        The sensor data (orientation, angular_velocity, linear_acceleration)
        passes through completely unchanged.
        """
        clean = self._clean_imu(msg)
        self.imu_pub.publish(clean)

    def _clean_imu(self, raw: Imu) -> Imu:
        """
        Returns a new Imu message with corrected frame_id and covariances.
        All sensor readings are copied unchanged from the raw message.

        Extracted as a pure function so unit tests can call it directly
        without needing a running publisher or subscriber.
        """
        clean = Imu()

        # --- Header ---
        # Keep the original timestamp — same reason as in tf2_broadcaster:
        # the timestamp says WHEN the measurement was taken.
        # Override only the frame_id so TF2 can locate the IMU in space.
        clean.header.stamp    = raw.header.stamp
        clean.header.frame_id = self.imu_frame_id   # 'imu_link'

        # --- Sensor data: pass through unchanged ---
        # We do NOT modify the actual measurements. The accelerometer
        # readings, gyro readings, and DMP orientation are already
        # in the IMU's own coordinate frame — correct as-is.
        clean.orientation           = raw.orientation
        clean.angular_velocity      = raw.angular_velocity
        clean.linear_acceleration   = raw.linear_acceleration

        # --- Covariances: replace zeros with realistic values ---
        # list() makes a copy — we don't want multiple messages sharing
        # the same list object.
        clean.orientation_covariance            = list(self.orientation_cov)
        clean.angular_velocity_covariance       = list(self.angular_vel_cov)
        clean.linear_acceleration_covariance    = list(self.linear_acc_cov)

        return clean

    @staticmethod
    def _diagonal_covariance(variance: float) -> list:
        """
        Builds a 3x3 diagonal covariance matrix (flattened to 9 values)
        where all three diagonal entries equal `variance`.

        A diagonal matrix means the three axes are assumed independent:
        uncertainty in x doesn't tell you anything about y or z.
        This is a reasonable assumption for an IMU in normal conditions.

              | var  0    0   |
          M = | 0    var  0   |  → [var, 0, 0, 0, var, 0, 0, 0, var]
              | 0    0    var |

        The EKF uses this matrix to weight how much to trust each axis.
        Larger variance = less trust = EKF relies more on other sensors.
        """
        return [
            variance, 0.0,      0.0,
            0.0,      variance, 0.0,
            0.0,      0.0,      variance,
        ]


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
