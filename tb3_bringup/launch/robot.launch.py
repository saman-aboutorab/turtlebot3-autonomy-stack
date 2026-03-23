"""
robot.launch.py
===============

Full robot bringup. One command starts every node needed to drive,
localise, and observe the TurtleBot3 Burger.

LAPTOP (fake_joints:=true, default):
  description.launch.py   — URDF + robot_state_publisher + joint_state_publisher
  sensors.launch.py       — odometry_publisher + imu_republisher
  ekf_node                — /odom + /imu/data → /odometry/filtered
  tf2_broadcaster         — /odometry/filtered → TF odom→base_footprint
  velocity_controller     — /cmd_vel_raw → /cmd_vel

REAL ROBOT (fake_joints:=false):
  hardware.launch.py      — turtlebot3_ros (OpenCR) + hlds_laser (LiDAR)
  description.launch.py   — URDF + robot_state_publisher (NO joint_state_publisher)
  sensors.launch.py       — odometry_publisher + imu_republisher
  ekf_node, tf2_broadcaster, velocity_controller (same as above)

WHY hardware.launch.py IS SEPARATE:
  The official turtlebot3_bringup starts its own robot_state_publisher and
  publishes /odom from turtlebot3_ros. Including it directly would create:
    - Two robot_state_publisher nodes with different URDFs → conflicting TF
    - Two /odom publishers → EKF receives interleaved messages from both
  hardware.launch.py starts only the hardware drivers with /odom remapped
  to /odom_hw so our odometry_publisher owns /odom cleanly.

ARGUMENTS:
  fake_joints (str, default: 'true')
    'true'  — laptop mode: joint_state_publisher fakes /joint_states
    'false' — robot mode: hardware.launch.py provides real sensor data

USAGE:
  ros2 launch tb3_bringup robot.launch.py                    # laptop
  ros2 launch tb3_bringup robot.launch.py fake_joints:=false # real robot
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
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
            'true  = laptop mode (joint_state_publisher fakes /joint_states)\n'
            'false = robot mode (hardware.launch.py provides real sensor data)'
        ),
    )

    # ── hardware drivers (real robot only) ───────────────────────────────────
    # Starts turtlebot3_ros (OpenCR) + hlds_laser (LiDAR).
    # Skipped on the laptop — joint_state_publisher handles /joint_states there.
    # UnlessCondition: include this when fake_joints is NOT true (i.e. real robot).
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'hardware.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('fake_joints')),
    )

    # ── URDF + robot_state_publisher ─────────────────────────────────────────
    # Forwards fake_joints → use_joint_state_publisher in description.launch.py.
    # On laptop (true):  joint_state_publisher starts alongside robot_state_publisher.
    # On robot  (false): joint_state_publisher is skipped; OpenCR provides /joint_states.
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'use_joint_state_publisher': LaunchConfiguration('fake_joints'),
        }.items(),
    )

    # ── sensor processing nodes ───────────────────────────────────────────────
    # odometry_publisher  — /joint_states → /odom
    # imu_republisher     — /imu          → /imu/data
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'sensors.launch.py')
        ),
    )

    # ── estimation + control nodes ────────────────────────────────────────────
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
        hardware_launch,
        description_launch,
        sensors_launch,
        ekf_node,
        tf2_broadcaster,
        velocity_controller,
    ])
