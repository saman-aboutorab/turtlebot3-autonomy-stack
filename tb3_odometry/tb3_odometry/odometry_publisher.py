#!/usr/bin/env python3
"""
odometry_publisher.py
=====================
Subscribes to /joint_states (wheel encoder angles from OpenCR),
integrates differential-drive kinematics, and publishes nav_msgs/Odometry
on /odom.

WHAT THIS NODE DOES:
  1. Receives wheel encoder positions from the OpenCR hardware driver
  2. Computes how much each wheel rotated since the last message
  3. Applies differential-drive kinematics to estimate robot motion
  4. Integrates motion into a running pose (x, y, heading)
  5. Publishes the pose + velocity as nav_msgs/Odometry on /odom

WHAT THIS NODE DOES NOT DO:
  - It does NOT broadcast TF2 transforms (that is tf2_broadcaster.py, Step 4)
  - It does NOT fuse with IMU (that is ekf_node.py, Step 6)
  - It does NOT know about the map (that is SLAM Toolbox, Step 9)

PARAMETERS (loaded from YAML in Step 8, hardcoded defaults here):
  wheel_radius      (float) default: 0.033  m  — TurtleBot3 Burger spec
  wheel_separation  (float) default: 0.160  m  — centre-to-centre
  odom_frame_id     (str)   default: 'odom'
  base_frame_id     (str)   default: 'base_footprint'

SUBSCRIPTIONS:
  /joint_states   sensor_msgs/JointState   — wheel encoder angles (radians)

PUBLICATIONS:
  /odom           nav_msgs/Odometry        — robot pose + velocity estimate

COORDINATE FRAME CONVENTION:
  The odom frame is fixed in the world. The robot starts at (0, 0, 0) in
  odom frame at startup. x=forward, y=left, θ=yaw (counter-clockwise positive).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        # ----------------------------------------------------------------
        # PARAMETERS
        # declare_parameter(name, default) registers a parameter that can
        # be overridden from YAML config or CLI at launch time.
        # We never hardcode these values anywhere else in the code.
        # ----------------------------------------------------------------
        self.declare_parameter('wheel_radius',     0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('odom_frame_id',    'odom')
        self.declare_parameter('base_frame_id',    'base_footprint')

        self.wheel_radius     = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.odom_frame_id    = self.get_parameter('odom_frame_id').value
        self.base_frame_id    = self.get_parameter('base_frame_id').value

        # ----------------------------------------------------------------
        # ROBOT STATE
        # These three variables are the complete 2D pose of the robot in
        # the odom frame. They start at zero (robot starts at odom origin)
        # and are updated every time we receive a /joint_states message.
        # ----------------------------------------------------------------
        self.x     = 0.0   # metres, forward from start
        self.y     = 0.0   # metres, left from start
        self.theta = 0.0   # radians, yaw angle (counter-clockwise positive)

        # Previous wheel positions — we store the last encoder reading
        # so we can compute the DELTA (how much it moved) each step.
        # None means "we haven't received any data yet".
        self.prev_left_pos  = None
        self.prev_right_pos = None

        # ----------------------------------------------------------------
        # SUBSCRIBER — /joint_states
        # QoS profile for sensor data: BEST_EFFORT reliability (we don't
        # need guaranteed delivery for high-rate sensor streams; if a
        # packet is lost, the next one arrives in ~20ms anyway).
        # ----------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            sensor_qos
        )

        # ----------------------------------------------------------------
        # PUBLISHER — /odom
        # RELIABLE QoS because the EKF (Step 6) and tf2_broadcaster
        # (Step 4) must not miss odometry messages.
        # ----------------------------------------------------------------
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10  # queue depth 10, default RELIABLE QoS
        )

        self.get_logger().info(
            f'OdometryPublisher started.\n'
            f'  wheel_radius:     {self.wheel_radius} m\n'
            f'  wheel_separation: {self.wheel_separation} m\n'
            f'  subscribing to:   /joint_states\n'
            f'  publishing to:    /odom'
        )

    # ====================================================================
    # CALLBACK — called every time /joint_states arrives (~50Hz on robot)
    # ====================================================================
    def joint_states_callback(self, msg: JointState):
        """
        Receives wheel encoder positions and integrates odometry.

        msg.name      — list of joint names, e.g. ['wheel_left_joint', 'wheel_right_joint']
        msg.position  — list of current angles in radians (same order as name)
        msg.velocity  — list of angular velocities in rad/s (same order)

        IMPORTANT: the order of joints in the name/position arrays is NOT
        guaranteed to be consistent. Always look up by name, not by index.
        """

        # --- Step A: Extract wheel positions by name ---
        # Build a dict {joint_name: position} from the parallel arrays.
        joint_positions = dict(zip(msg.name, msg.position))
        joint_velocities = dict(zip(msg.name, msg.velocity))

        # Check both wheels are present (other joints may appear in future)
        if 'wheel_left_joint'  not in joint_positions or \
           'wheel_right_joint' not in joint_positions:
            self.get_logger().warn(
                'wheel joints not found in /joint_states. '
                f'Got: {list(joint_positions.keys())}'
            )
            return

        left_pos  = joint_positions['wheel_left_joint']
        right_pos = joint_positions['wheel_right_joint']

        # --- Step B: Initialise on first message ---
        # We cannot compute a delta on the very first callback because
        # there is no "previous" position. Store and return.
        if self.prev_left_pos is None:
            self.prev_left_pos  = left_pos
            self.prev_right_pos = right_pos
            self.get_logger().info('First /joint_states received. Odometry initialised.')
            return

        # --- Step C: Compute wheel angle deltas ---
        # How many radians did each wheel rotate since last callback?
        delta_left  = left_pos  - self.prev_left_pos
        delta_right = right_pos - self.prev_right_pos

        self.prev_left_pos  = left_pos
        self.prev_right_pos = right_pos

        # --- Step D: Convert angle deltas to arc lengths ---
        # arc_length = angle_radians * wheel_radius
        # This is the distance each wheel's contact patch traveled along the floor.
        dist_left  = delta_left  * self.wheel_radius
        dist_right = delta_right * self.wheel_radius

        # --- Step E: Differential-drive kinematics ---
        # Linear displacement of the robot centre (average of both wheels)
        linear_dist = (dist_right + dist_left) / 2.0

        # Angular displacement of the robot body
        # positive = counter-clockwise (left turn)
        angular_dist = (dist_right - dist_left) / self.wheel_separation

        # --- Step F: Integrate pose ---
        # Project the linear displacement along the robot's current heading.
        # We use the MIDPOINT heading (theta + angular_dist/2) for better
        # accuracy — this is the Runge-Kutta 2nd order approximation.
        mid_theta = self.theta + angular_dist / 2.0

        self.x     += linear_dist * math.cos(mid_theta)
        self.y     += linear_dist * math.sin(mid_theta)
        self.theta += angular_dist

        # Normalise theta to [-π, π] to prevent floating-point accumulation
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # --- Step G: Compute velocities ---
        # We need the time delta to convert distance to velocity.
        # Use the message's own timestamp for accuracy (not wall-clock time).
        stamp = msg.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        # For velocity, use the joint velocity field directly if available,
        # otherwise fall back to the raw joint velocities from the message.
        left_vel  = joint_velocities.get('wheel_left_joint',  0.0)
        right_vel = joint_velocities.get('wheel_right_joint', 0.0)

        linear_vel  = (right_vel + left_vel)  / 2.0 * self.wheel_radius
        angular_vel = (right_vel - left_vel) / self.wheel_separation * self.wheel_radius

        # --- Step H: Build and publish Odometry message ---
        self._publish_odometry(current_time, linear_vel, angular_vel)

    def _publish_odometry(self, timestamp_sec: float, linear_vel: float, angular_vel: float):
        """
        Constructs and publishes a nav_msgs/Odometry message.

        The Odometry message has two main sections:
          pose  — WHERE the robot is (position + orientation + covariance)
          twist — HOW FAST it's moving (linear + angular velocity + covariance)
        """
        msg = Odometry()

        # --- Header ---
        # frame_id = the reference frame this POSE is expressed in.
        # child_frame_id = the frame being described (the robot body).
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame_id    # 'odom'
        msg.child_frame_id  = self.base_frame_id    # 'base_footprint'

        # --- Pose: position ---
        msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)

        # --- Pose: orientation ---
        # Nav2 and RViz2 expect orientation as a quaternion, not an Euler angle.
        # For 2D motion (only yaw changes), the quaternion simplifies to:
        #   qx=0, qy=0, qz=sin(θ/2), qw=cos(θ/2)
        msg.pose.pose.orientation = self._yaw_to_quaternion(self.theta)

        # --- Pose covariance ---
        # A 6x6 matrix flattened to 36 values (row-major).
        # Order: [x, y, z, roll, pitch, yaw]
        # Diagonal values = variance for each DOF.
        # Off-diagonal = correlations (we assume independent → zeros).
        # The robot moves in 2D so z/roll/pitch are known exactly → near-zero.
        # x, y, yaw get realistic uncertainty values that the EKF will use.
        msg.pose.covariance = [
            1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,   # x variance
            0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,   # y variance
            0.0,  0.0,  1e-6, 0.0,  0.0,  0.0,   # z (not used in 2D)
            0.0,  0.0,  0.0,  1e-6, 0.0,  0.0,   # roll (not used)
            0.0,  0.0,  0.0,  0.0,  1e-6, 0.0,   # pitch (not used)
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-2,  # yaw variance
        ]

        # --- Twist: velocity ---
        # Expressed in the ROBOT frame (base_footprint), not odom.
        # For a differential drive: only forward (x) and yaw (z) are non-zero.
        msg.twist.twist.linear  = Vector3(x=linear_vel, y=0.0, z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=angular_vel)

        # --- Twist covariance ---
        msg.twist.covariance = [
            1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e-6, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e-6, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e-6, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e-6, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-2,
        ]

        self.odom_pub.publish(msg)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """
        Converts a yaw angle (radians) to a geometry_msgs/Quaternion.

        For 2D motion, only the z-axis rotation is non-zero.
        The full quaternion for a pure yaw rotation is:
          qx = 0
          qy = 0
          qz = sin(yaw / 2)
          qw = cos(yaw / 2)

        This avoids a dependency on tf_transformations for a trivial case.
        """
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
