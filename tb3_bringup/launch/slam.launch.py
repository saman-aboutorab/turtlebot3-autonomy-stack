"""
slam.launch.py
==============

Starts the full robot stack PLUS slam_toolbox for building a 2D map.

This is the launch file you run when you want to drive the robot around
and build a map you can save and reuse for autonomous navigation (Step 10).

Stack started by this file:
  robot.launch.py (description + sensors + ekf + tf2 + velocity_controller)
    └── slam_toolbox (async_slam_toolbox_node)
          └── reads config/burger.yaml for algorithm parameters

HOW SLAM TOOLBOX FITS IN:
  slam_toolbox subscribes to:
    /scan              (sensor_msgs/LaserScan)   — LiDAR point cloud
    /tf                (the TF tree)              — needs odom→base_footprint
                                                     and base_footprint→base_scan

  It publishes:
    /map               (nav_msgs/OccupancyGrid)   — the 2D map (0=free, 100=occupied)
    /tf                map→odom transform          — closes the localisation loop

  The robot's LiDAR (LDS-01) publishes /scan via the OpenCR firmware.
  Our TF tree (from Steps 2–6) provides the transforms slam_toolbox needs.

ASYNC vs SYNC:
  async_slam_toolbox_node processes scans asynchronously. If the robot moves
  faster than scans can be processed, some scans are dropped — acceptable for
  mapping at normal driving speeds. sync_slam_toolbox_node processes every
  scan but can fall behind; avoid it on the RPi4.

ARGUMENTS:
  fake_joints (str, default: 'true')   forwarded to robot.launch.py
  slam_params_file (str, default: burger.yaml path)

USAGE:
  # On the real robot — build a map
  ros2 launch tb3_bringup slam.launch.py fake_joints:=false

  # On the laptop — structure test only (no LiDAR = no map, but nodes start)
  ros2 launch tb3_bringup slam.launch.py

SAVE THE MAP (after driving):
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
  # Creates my_map.pgm (image) + my_map.yaml (metadata)
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

    default_slam_params = os.path.join(pkg, 'config', 'burger.yaml')

    # ── arguments ────────────────────────────────────────────────────────────
    fake_joints_arg = DeclareLaunchArgument(
        name='fake_joints',
        default_value='true',
        description='true = laptop (fake joints); false = real robot',
    )

    slam_params_arg = DeclareLaunchArgument(
        name='slam_params_file',
        default_value=default_slam_params,
        description='Full path to the slam_toolbox parameter YAML file',
    )

    # ── robot bringup (includes everything from Steps 2–7) ───────────────────
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'fake_joints': LaunchConfiguration('fake_joints'),
        }.items(),
    )

    # ── slam_toolbox ─────────────────────────────────────────────────────────
    # async_slam_toolbox_node: processes scans asynchronously.
    # parameters= loads burger.yaml which contains the ros__parameters block
    # under the 'slam_toolbox' node name (ROS2 YAML parameter convention).
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file')],
    )

    return LaunchDescription([
        fake_joints_arg,
        slam_params_arg,
        robot_launch,
        slam_toolbox,
    ])
