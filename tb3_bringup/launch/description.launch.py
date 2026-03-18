"""
description.launch.py
======================
Launches the robot description (URDF) via robot_state_publisher.

WHAT THIS LAUNCH FILE DOES:
  1. Finds the URDF xacro file via get_package_share_directory()
  2. Runs `xacro` to expand all <xacro:property> macros -> produces plain URDF XML
  3. Passes the resulting XML string to robot_state_publisher as a parameter
  4. robot_state_publisher (a C++ node) reads the XML and publishes:
       - /robot_description topic (the raw URDF string, used by RViz2)
       - Static TF2 transforms for every <joint type="fixed"> in the URDF
       - Dynamic TF2 transform updates when it receives /joint_states

HOW THE C++ SIDE WORKS (robot_state_publisher internals):
  robot_state_publisher is a well-tested C++ ROS2 node from Robotis/OSRF.
  It uses the KDL (Kinematics and Dynamics Library) to parse the URDF XML,
  extract the kinematic chain, and efficiently broadcast TF2 transforms.
  Writing this from scratch in C++ ourselves is unnecessary -- this is the
  right tool for the job. Our C++ nodes come in Steps 3, 4, 7, 12.

WHY get_package_share_directory() NOT __file__:
  After `colcon build`, all launch files are installed (or symlinked) into:
    install/<package>/share/<package>/launch/
  Using __file__ or os.getcwd() is fragile because the file is no longer
  next to the source tree. get_package_share_directory('tb3_bringup') always
  returns the correct installed path regardless of where you run the command.
  The URDF is installed there too via CMakeLists.txt:
    install(DIRECTORY ../hardware DESTINATION share/tb3_bringup)
  -> install/tb3_bringup/share/tb3_bringup/hardware/urdf/...

USAGE:
  ros2 launch tb3_bringup description.launch.py
  ros2 launch tb3_bringup description.launch.py urdf_file:=/path/to/custom.urdf.xacro
  ros2 launch tb3_bringup description.launch.py use_joint_state_publisher:=false

ARGUMENTS:
  use_joint_state_publisher (bool, default: true)
    true  — start joint_state_publisher (laptop / no hardware)
    false — skip it; the OpenCR provides /joint_states on the real robot

VERIFY:
  ros2 run tf2_tools view_frames          # generates a PDF of the TF tree
  ros2 topic echo /robot_description      # see the raw URDF XML
  ros2 run tf2_ros tf2_echo base_link base_scan   # see the transform
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --- 1. Locate the URDF xacro file ---
    # get_package_share_directory('tb3_bringup') returns:
    #   install/tb3_bringup/share/tb3_bringup/
    # hardware/ is installed there by CMakeLists.txt:
    #   install(DIRECTORY ../hardware DESTINATION share/${PROJECT_NAME})
    # So the URDF ends up at:
    #   install/tb3_bringup/share/tb3_bringup/hardware/urdf/turtlebot3_burger_sensors.urdf.xacro
    pkg_share = get_package_share_directory('tb3_bringup')
    default_urdf = os.path.join(
        pkg_share, 'hardware', 'urdf', 'turtlebot3_burger_sensors.urdf.xacro'
    )

    # --- 2. Declare launch arguments ---
    urdf_file_arg = DeclareLaunchArgument(
        name='urdf_file',
        default_value=default_urdf,
        description='Absolute path to the robot URDF xacro file'
    )

    # Controls whether joint_state_publisher starts.
    # On the real robot the OpenCR provides /joint_states, so we don't want
    # the fake publisher fighting it. robot.launch.py passes false here.
    use_jsp_arg = DeclareLaunchArgument(
        name='use_joint_state_publisher',
        default_value='true',
        description='true = laptop (fake joints); false = real robot (OpenCR provides /joint_states)'
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
            # ParameterValue(..., value_type=str) tells the launch system not to
            # try to parse the xacro output as YAML -- treat it as a raw string.
            # Command() runs `xacro <file>` and captures stdout as the value.
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('urdf_file')]),
                value_type=str
            ),
            # Publish TF at 50Hz -- matches our future odometry rate
            'publish_frequency': 50.0,
        }]
    )

    # --- 4. joint_state_publisher ---
    # On the laptop: publishes /joint_states at position 0 so RViz can show
    # the robot. On the real robot, OpenCR provides /joint_states — starting
    # this node there would publish conflicting messages on the same topic.
    # IfCondition reads the use_joint_state_publisher argument at launch time.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
    )

    return LaunchDescription([
        urdf_file_arg,
        use_jsp_arg,
        robot_state_publisher,
        joint_state_publisher,
    ])
