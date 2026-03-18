"""
robot.launch.py
===============

Full robot bringup. One command starts every node needed to drive,
localise, and observe the TurtleBot3 Burger.

This file composes three pieces:

  1. description.launch.py  — URDF → robot_state_publisher → static TF
                               [+ joint_state_publisher when fake_joints=true]
  2. sensors.launch.py      — odometry_publisher + imu_republisher
  3. Three nodes started directly here:
       ekf_node              — fuses /odom + /imu/data → /odometry/filtered
       tf2_broadcaster       — /odometry/filtered → TF odom→base_footprint
       velocity_controller   — /cmd_vel_raw → clamp/ramp/timeout → /cmd_vel

COMPOSITION PATTERN:
  IncludeLaunchDescription + PythonLaunchDescriptionSource is the ROS2 way
  to nest launch files. Arguments can be forwarded to included files via
  launch_arguments={...}.items(). This is equivalent to calling the other
  launch file as a function with keyword arguments.

ARGUMENTS:
  fake_joints (str, default: 'true')
    'true'  — run joint_state_publisher (laptop testing, no OpenCR)
    'false' — skip it; the real robot's OpenCR provides /joint_states

USAGE:
  # Laptop (default: fake joints for testing)
  ros2 launch tb3_bringup robot.launch.py

  # Real robot (no fake joints; OpenCR provides /joint_states and /imu)
  ros2 launch tb3_bringup robot.launch.py fake_joints:=false

VERIFY:
  ros2 node list                          # should show all 7 nodes
  ros2 topic list                         # /odom, /imu/data, /odometry/filtered, /tf ...
  ros2 run tf2_tools view_frames          # full TF tree as a PDF
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('tb3_bringup')

    # ── arguments ────────────────────────────────────────────────────────────
    fake_joints_arg = DeclareLaunchArgument(
        name='fake_joints',
        default_value='true',
        description=(
            'true  = start joint_state_publisher (laptop, no hardware)\n'
            'false = rely on OpenCR for /joint_states (real robot)'
        ),
    )

    # ── included launch files ─────────────────────────────────────────────────

    # URDF + robot_state_publisher + (conditionally) joint_state_publisher.
    # We forward fake_joints as use_joint_state_publisher so description.launch.py
    # knows whether to start the fake publisher.
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'use_joint_state_publisher': LaunchConfiguration('fake_joints'),
        }.items(),
    )

    # odometry_publisher + imu_republisher
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'sensors.launch.py')
        ),
    )

    # ── nodes started directly ────────────────────────────────────────────────

    ekf_node = Node(
        package='tb3_odometry',
        executable='ekf_node.py',
        name='ekf_node',
        output='screen',
    )

    tf2_broadcaster = Node(
        package='tb3_odometry',
        executable='tf2_broadcaster.py',
        name='tf2_broadcaster',
        output='screen',
    )

    velocity_controller = Node(
        package='tb3_odometry',
        executable='velocity_controller.py',
        name='velocity_controller',
        output='screen',
    )

    return LaunchDescription([
        fake_joints_arg,
        description_launch,
        sensors_launch,
        ekf_node,
        tf2_broadcaster,
        velocity_controller,
    ])
