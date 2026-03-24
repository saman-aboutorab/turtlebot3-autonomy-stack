"""
cartographer.launch.py
======================

Starts the full robot stack PLUS Google Cartographer for building a 2D map.

Replaces slam.launch.py. Use this launch file instead — slam_toolbox's arm64
apt binary crashes on the RPi4 (see PROGRESS.md [1-9d]).

Stack started by this file:
  robot.launch.py (description + sensors + ekf + tf2 + velocity_controller)
    └── cartographer_node     — 2D SLAM: /scan + /tf → /map + map→odom TF
    └── occupancy_grid_node   — converts cartographer's submap list to /map

HOW CARTOGRAPHER DIFFERS FROM SLAM_TOOLBOX:
  cartographer_node does NOT publish a nav_msgs/OccupancyGrid on /map directly.
  It publishes submaps internally. occupancy_grid_node subscribes to those submaps
  and periodically stitches them into a single /map message that RViz and Nav2
  can consume.

  This two-node design means there is a short delay (~1 s) before the first /map
  message appears in RViz — this is normal.

ARGUMENTS:
  fake_joints (str, default: 'true')   forwarded to robot.launch.py

USAGE:
  # On the real robot — build a map
  ros2 launch tb3_bringup cartographer.launch.py fake_joints:=false

  # On the laptop — structure test only (no LiDAR = no map, but nodes start)
  ros2 launch tb3_bringup cartographer.launch.py

SAVE THE MAP (after driving):
  ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map
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

    lua_config = os.path.join(pkg, 'config', 'burger_cartographer.lua')

    # ── arguments ────────────────────────────────────────────────────────────
    fake_joints_arg = DeclareLaunchArgument(
        name='fake_joints',
        default_value='true',
        description='true = laptop (fake joints); false = real robot',
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

    # ── cartographer SLAM node ────────────────────────────────────────────────
    # cartographer_node performs the actual SLAM:
    #   - subscribes to /scan and /odom (via TF)
    #   - maintains a pose graph of submaps
    #   - publishes map→odom TF (closes the localisation loop)
    #
    # configuration_directory and configuration_basename tell cartographer
    # where to find the Lua config file. The basename must NOT include .lua.
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(pkg, 'config'),
            '-configuration_basename', 'burger_cartographer.lua',
        ],
        remappings=[
            # Cartographer defaults to 'scan' — remap to our LiDAR topic '/scan'
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )

    # ── occupancy grid node ───────────────────────────────────────────────────
    # Converts cartographer's internal submap list into a nav_msgs/OccupancyGrid
    # on /map, which RViz and Nav2 can consume.
    # resolution: grid cell size in metres (must match burger_cartographer.lua)
    # publish_period_sec: how often to re-stitch and publish the full map
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
        }],
    )

    return LaunchDescription([
        fake_joints_arg,
        robot_launch,
        cartographer_node,
        occupancy_grid_node,
    ])
