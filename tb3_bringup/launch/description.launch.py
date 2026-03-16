"""
description.launch.py
======================
Launches the robot description (URDF) via robot_state_publisher.

WHAT THIS LAUNCH FILE DOES:
  1. Finds the URDF xacro file in the hardware/ directory
  2. Runs `xacro` to expand all <xacro:property> macros → produces plain URDF XML
  3. Passes the resulting XML string to robot_state_publisher as a parameter
  4. robot_state_publisher (a C++ node) reads the XML and publishes:
       - /robot_description topic (the raw URDF string, used by RViz2)
       - Static TF2 transforms for every <joint type="fixed"> in the URDF
       - Dynamic TF2 transform updates when it receives /joint_states

HOW THE C++ SIDE WORKS (robot_state_publisher internals):
  robot_state_publisher is a well-tested C++ ROS2 node from Robotis/OSRF.
  It uses the KDL (Kinematics and Dynamics Library) to parse the URDF XML,
  extract the kinematic chain, and efficiently broadcast TF2 transforms.
  Writing this from scratch in C++ ourselves is unnecessary — this is the
  right tool for the job. Our C++ nodes come in Steps 3, 4, 7, 12.

USAGE:
  ros2 launch tb3_bringup description.launch.py
  ros2 launch tb3_bringup description.launch.py urdf_file:=/path/to/custom.urdf.xacro

VERIFY:
  ros2 run tf2_tools view_frames          # generates a PDF of the TF tree
  ros2 topic echo /robot_description      # see the raw URDF XML
  ros2 run tf2_ros tf2_echo base_link base_scan   # see the transform
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # --- 1. Locate the URDF xacro file ---
    # get_package_share_directory finds the installed share/ location for a package.
    # After `colcon build`, files in install() directives end up under:
    #   install/<package>/share/<package>/...
    # For hardware/, we use the workspace path directly since hardware/ is not
    # a ROS package — it doesn't have a package.xml.
    pkg_dir = os.path.join(
        os.path.dirname(__file__),   # this launch file's directory
        '..', '..', '..',            # up to workspace root
        'hardware', 'urdf',
        'turtlebot3_burger_sensors.urdf.xacro'
    )
    default_urdf = os.path.normpath(pkg_dir)

    # --- 2. Declare a launch argument so the URDF path can be overridden ---
    # This is the ROS2 way to make launch files configurable from CLI:
    #   ros2 launch tb3_bringup description.launch.py urdf_file:=/other/robot.urdf.xacro
    urdf_file_arg = DeclareLaunchArgument(
        name='urdf_file',
        default_value=default_urdf,
        description='Absolute path to the robot URDF xacro file'
    )

    # --- 3. robot_state_publisher node ---
    # Command() runs `xacro <file>` at launch time and captures its stdout.
    # This expands all xacro:property macros and produces plain URDF XML.
    # That XML string becomes the robot_description parameter.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # Command() evaluates at launch time: runs `xacro <urdf_file>`
            'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')]),
            # Publish TF at 50Hz — matches our future odometry rate
            'publish_frequency': 50.0,
        }]
    )

    # --- 4. joint_state_publisher ---
    # For a real robot, /joint_states comes from the OpenCR hardware driver.
    # On the laptop (no hardware), we use joint_state_publisher to fake it
    # so robot_state_publisher can still update the wheel joint positions.
    # When we run on the real robot, this node is replaced by the OpenCR driver.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    return LaunchDescription([
        urdf_file_arg,
        robot_state_publisher,
        joint_state_publisher,
    ])
