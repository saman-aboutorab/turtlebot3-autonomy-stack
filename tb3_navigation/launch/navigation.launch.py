"""
navigation.launch.py
====================

Starts the full robot stack + Nav2 for autonomous point-to-point navigation
using a pre-built map (built with cartographer.launch.py in Step 9).

WHAT THIS STARTS:
  robot.launch.py  (Steps 2–7: URDF, odometry, TF2, EKF, velocity_controller)
    └── map_server        — loads the saved .pgm/.yaml map, publishes /map
    └── amcl              — localises robot in the map, publishes map→odom TF
    └── planner_server    — global path planning (NavFn A*)
    └── controller_server — local path following (Regulated Pure Pursuit)
    └── bt_navigator      — behaviour tree: plan → follow → recover
    └── waypoint_follower — Nav2 built-in sequential goal server (used in Step 11)
    └── lifecycle_manager — activates Nav2 nodes in the correct order

KEY DIFFERENCE FROM SLAM:
  During SLAM (cartographer.launch.py): cartographer publishes the map→odom TF.
  During navigation (this file): AMCL publishes map→odom TF instead.
  Never run both at the same time — they would publish conflicting TF transforms.

DATA FLOW:
  my_room.yaml/pgm ──► map_server ──► /map ──► global_costmap (static layer)
  /scan ────────────► amcl ──────────────────► map→odom TF
  /scan ────────────► local_costmap (obstacle layer)
  goal pose ────────► bt_navigator ──► planner ──► controller ──► /cmd_vel_raw
                                                                         │
                                                          velocity_controller (Step 7)
                                                                         │
                                                                   turtlebot3_ros
                                                                         │
                                                                      motors

ARGUMENTS:
  fake_joints (str, default: 'true')           forwarded to robot.launch.py
  map_file    (str, default: maps/my_room.yaml) full path to map YAML

USAGE:
  # Real robot
  ros2 launch tb3_navigation navigation.launch.py fake_joints:=false

  # Laptop — structure test only (no real sensors, nav2 starts but won't plan)
  ros2 launch tb3_navigation navigation.launch.py

SEND A GOAL:
  # Option 1: RViz2 — click "2D Goal Pose", click location on the map
  # Option 2: Python node
  ros2 run tb3_navigation nav_goal_sender.py --ros-args -p goal_x:=1.0 -p goal_y:=0.5

INITIAL POSE:
  AMCL needs an initial pose estimate to localise.
  In RViz2: click "2D Pose Estimate" and mark where the robot is on the map.
  Without it, AMCL starts at the map origin and may take longer to converge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_nav = get_package_share_directory('tb3_navigation')
    pkg_bringup = get_package_share_directory('tb3_bringup')

    default_map_file = os.path.join(pkg_nav, 'maps', 'my_room.yaml')
    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # ── arguments ────────────────────────────────────────────────────────────
    fake_joints_arg = DeclareLaunchArgument(
        name='fake_joints',
        default_value='true',
        description='true = laptop (fake joints); false = real robot',
    )

    map_file_arg = DeclareLaunchArgument(
        name='map_file',
        default_value=default_map_file,
        description='Full path to the map YAML file to load',
    )

    # ── robot bringup ─────────────────────────────────────────────────────────
    # Includes: turtlebot3_ros (OpenCR + LiDAR), robot_state_publisher,
    # odometry_publisher, imu_republisher, ekf_node, tf2_broadcaster,
    # velocity_controller.
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'fake_joints': LaunchConfiguration('fake_joints'),
        }.items(),
    )

    # ── map_server ────────────────────────────────────────────────────────────
    # Lifecycle node. Loads my_room.pgm + my_room.yaml from disk.
    # Publishes the occupancy grid on /map.
    # yaml_filename is overridden here at launch time — nav2_params.yaml
    # leaves it empty as a placeholder.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'yaml_filename': LaunchConfiguration('map_file')},
        ],
    )

    # ── amcl ──────────────────────────────────────────────────────────────────
    # Lifecycle node. Subscribes to /scan and /tf.
    # Publishes map→odom TF, replacing cartographer during navigation.
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ── planner_server ────────────────────────────────────────────────────────
    # Lifecycle node. Exposes the ComputePathToPose action server.
    # Uses NavFn (A*) to search the global costmap for a collision-free path.
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ── controller_server ─────────────────────────────────────────────────────
    # Lifecycle node. Exposes the FollowPath action server.
    # Uses Regulated Pure Pursuit to follow the global path at 20 Hz.
    # Publishes to /cmd_vel_raw (not /cmd_vel) so commands flow through
    # our velocity_controller (Step 7) for clamping and ramping.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', '/cmd_vel_raw'),
        ],
    )

    # ── behavior_server ───────────────────────────────────────────────────────
    # Lifecycle node. Provides recovery action servers required by the default
    # Nav2 BT XML (navigate_to_pose_w_replanning_and_recovery.xml):
    #   spin, backup, wait, drive_on_heading
    # Without this, bt_navigator fails to activate with:
    #   "spin action server not available"
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ── bt_navigator ──────────────────────────────────────────────────────────
    # Lifecycle node. Exposes the NavigateToPose action server.
    # Orchestrates: ComputePathToPose → FollowPath → Recovery behaviours.
    # nav_goal_sender.py (Step 10) and waypoint_follower.py (Step 11) both
    # call this action server.
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ── waypoint_follower ─────────────────────────────────────────────────────
    # Lifecycle node. Nav2 built-in server for sequential goal execution.
    # Not called directly in Step 10 (single goal), but must be active
    # because lifecycle_manager expects it in the node list.
    # Our custom waypoint_follower.py (Step 11) will call this server.
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ── lifecycle_manager ─────────────────────────────────────────────────────
    # Manages Nav2 node lifecycle transitions in order.
    # behavior_server must be active before bt_navigator so that the spin/
    # backup/wait action servers are available when the BT XML is loaded.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    return LaunchDescription([
        fake_joints_arg,
        map_file_arg,
        robot_launch,
        map_server,
        amcl,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])
