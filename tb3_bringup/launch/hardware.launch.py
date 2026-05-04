"""
hardware.launch.py
==================

Starts ONLY the hardware interface nodes for the TurtleBot3 Burger.
Intended to be included by robot.launch.py when fake_joints:=false.

WHY THIS FILE EXISTS (and why we don't use turtlebot3_bringup robot.launch.py):
  The official turtlebot3_bringup robot.launch.py starts three things:
    1. robot_state_publisher  — we have our own, with our custom URDF
    2. LiDAR driver           — we need this
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

WHAT rplidar_composition DOES (RPLIDAR C1, current hardware):
  - Opens /dev/ttyUSB0 at 460800 baud (STM32 USB chip on the C1 adapter board)
  - Reads 360° distance measurements → publishes /scan (sensor_msgs/LaserScan)
  - Range: 0.05–12 m, 10 Hz, 2.1K points/scan, frame_id=base_scan
  - angle_compensate=true: interpolates points to uniform angular spacing

  Previous LiDAR (LDS-01, hls_lfcd_lds_driver / hlds_laser_publisher):
    - Connected via /dev/ttyUSB0 at 230400 baud (CP2102 USB chip)
    - Range: 0.12–3.5 m, 5 Hz — replaced due to internal UART hardware fault
    (broken ribbon cable between sensor PCB and USB board, zero 0xFA bytes
    at all baud rates, confirmed 2026-03-24 — see PROGRESS.md [1-9e])

  ROBOTIS LDS-03 (attempted replacement, also failed):
    - Tx-only USART at 115200 baud, USB2LDS adapter (CP2102 chip)
    - Confirmed hardware fault on unit received: 0 bytes at all tests
    (see PROGRESS.md [1-9f])

DATA FLOW ON REAL ROBOT:
  OpenCR ──── turtlebot3_ros ──► /joint_states ──► odometry_publisher → /odom
                             ──► /imu          ──► imu_republisher   → /imu/data
                             ◄── /cmd_vel      ◄── velocity_controller
  LiDAR ───── rplidar_node   ──► /scan         ──► cartographer
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # turtlebot3_ros requires opencr.id and other hardware parameters.
    # These are defined in the burger.yaml shipped with turtlebot3_bringup.
    # Without this file the node crashes with UninitializedStaticallyTypedParameterException.
    # The serial port is passed as a CLI argument (-i), not as a ROS parameter.
    tb3_params = os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'param',
        'burger.yaml',
    )

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
        parameters=[
            tb3_params,
            {
                # --- required static params not set in burger.yaml for this node ---
                'namespace': '',

                # burger.yaml puts odometry.* under 'diff_drive_controller', not
                # 'turtlebot3_node', so turtlebot3_ros never receives them from the file.
                # We set them explicitly here.
                'odometry.frame_id':       'odom',
                'odometry.child_frame_id': 'base_footprint',
                'odometry.use_imu':        True,
                # False: our tf2_broadcaster publishes odom→base_footprint from the
                # filtered odometry. If turtlebot3_ros also published this TF we'd
                # get two conflicting transforms on /tf.
                'odometry.publish_tf':     False,

                # burger.yaml sets enable_stamped_cmd_vel: true, which makes
                # turtlebot3_ros expect geometry_msgs/TwistStamped on /cmd_vel.
                # Our velocity_controller publishes plain geometry_msgs/Twist.
                # Override to false so our commands are accepted.
                'enable_stamped_cmd_vel':  False,
            },
        ],
        arguments=['-i', '/dev/ttyACM0'],
        remappings=[
            # Hide their odometry so our odometry_publisher.py owns /odom cleanly.
            ('odom', 'odom_hw'),
        ],
    )

    # ── LiDAR driver ─────────────────────────────────────────────────────────
    # Slamtec RPLIDAR C1 — current hardware as of 2026-05-04.
    # rplidar_composition reads scans over /dev/ttyUSB0 at 460800 baud.
    # Publishes /scan (sensor_msgs/LaserScan, frame_id=base_scan).
    # angle_compensate=True: interpolates points to uniform angular spacing,
    #   required for cartographer to correctly interpret the scan geometry.
    # scan_mode is left unset: the driver auto-selects "Standard" mode for C1.
    #   (Explicitly setting scan_mode:=Standard fails with SDK 1.12.0 on C1
    #    because the deprecated checkExpressScanSupported API is called first.)
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port':     '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id':        'base_scan',
            'angle_compensate': True,
        }],
    )

    # ── Previous LiDAR drivers (kept for reference) ───────────────────────────
    # LDS-01 driver (hls_lfcd_lds_driver) — hardware fault, retired 2026-03-24.
    # Range: 0.12–3.5 m, 5 Hz, 230400 baud, CP2102 USB chip.
    # Broken internal UART ribbon cable → zero 0xFA bytes at all baud rates.
    #
    # lidar_node = Node(
    #     package='hls_lfcd_lds_driver',
    #     executable='hlds_laser_publisher',
    #     name='hlds_laser_publisher',
    #     output='screen',
    #     parameters=[{
    #         'port':     '/dev/ttyUSB0',
    #         'frame_id': 'base_scan',
    #     }],
    # )
    #
    # LDS-03 driver (ld08_driver) — hardware fault on unit received, retired 2026-05-04.
    # Range: 0.05–12 m, 10 Hz, 115200 baud, Tx-only USART, USB2LDS adapter.
    # Unit produced zero bytes at all serial tests — hardware DOA.
    #
    # lidar_node = Node(
    #     package='ld08_driver',
    #     executable='ld08_driver',
    #     name='ld08_driver',
    #     output='screen',
    #     parameters=[{
    #         'port':     '/dev/ttyUSB0',
    #         'frame_id': 'base_scan',
    #     }],
    # )

    return LaunchDescription([
        turtlebot3_node,
        lidar_node,
    ])
