"""
hardware.launch.py
==================

Starts ONLY the hardware interface nodes for the TurtleBot3 Burger.
Intended to be included by robot.launch.py when fake_joints:=false.

WHY THIS FILE EXISTS (and why we don't use turtlebot3_bringup robot.launch.py):
  The official turtlebot3_bringup robot.launch.py starts three things:
    1. robot_state_publisher  — we have our own, with our custom URDF
    2. hlds_laser_publisher   — LiDAR driver, we need this
    3. turtlebot3_ros          — OpenCR interface, we need this

  If we ran both the official bringup AND our robot.launch.py:
    - Two robot_state_publisher nodes would publish conflicting TF trees
    - turtlebot3_ros publishes /odom, and so does our odometry_publisher.py
      → the EKF would receive interleaved messages from both publishers

  This file starts (2) and (3) only, with one remap:
    turtlebot3_ros /odom → /odom_hw
  so it doesn't collide with our odometry_publisher's /odom output.

WHAT turtlebot3_ros DOES:
  - Opens /dev/ttyACM0 (serial to OpenCR)
  - Reads Dynamixel motor encoders → publishes /joint_states
  - Reads MPU-9250 IMU             → publishes /imu
  - Reads battery / sensor state   → publishes /battery_state, /sensor_state
  - Subscribes to /cmd_vel         → sends velocity commands to the motors

WHAT hlds_laser_publisher DOES:
  - Opens /dev/ttyUSB0 (serial to LDS-01 LiDAR)
  - Reads distance measurements    → publishes /scan (sensor_msgs/LaserScan)

DATA FLOW ON REAL ROBOT:
  OpenCR ──── turtlebot3_ros ──► /joint_states ──► odometry_publisher → /odom
                             ──► /imu          ──► imu_republisher   → /imu/data
                             ◄── /cmd_vel      ◄── velocity_controller
  LiDAR ───── hlds_laser     ──► /scan         ──► slam_toolbox
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ── OpenCR hardware interface ─────────────────────────────────────────────
    # turtlebot3_ros talks to the OpenCR over /dev/ttyACM0.
    # It publishes /joint_states, /imu, /battery_state, /sensor_state.
    # It also publishes /odom internally — we remap that to /odom_hw so it
    # does not conflict with our odometry_publisher which publishes on /odom.
    turtlebot3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        name='turtlebot3_node',
        output='screen',
        parameters=[{
            'opencr_port': '/dev/ttyACM0',
            'baud_rate': 1000000,
        }],
        remappings=[
            # Hide their odometry so our odometry_publisher.py owns /odom cleanly.
            ('odom', 'odom_hw'),
        ],
    )

    # ── LiDAR driver ─────────────────────────────────────────────────────────
    # hlds_laser_publisher reads the LDS-01 over /dev/ttyUSB0.
    # Publishes /scan (sensor_msgs/LaserScan, frame_id=base_scan).
    lidar_node = Node(
        package='hls_lfcd_lds_driver',
        executable='hlds_laser_publisher',
        name='hlds_laser_publisher',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'frame_id': 'base_scan',
        }],
    )

    return LaunchDescription([
        turtlebot3_node,
        lidar_node,
    ])
