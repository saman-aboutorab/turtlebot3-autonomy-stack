"""
teleop.launch.py
================

Starts keyboard teleoperation for manually driving the robot.

Run this in a SEPARATE terminal while robot.launch.py (or slam.launch.py)
is already running in another terminal. This gives you keyboard control
of the robot through the velocity controller's safety layers.

WHY NOT PUBLISH DIRECTLY TO /cmd_vel:
  teleop_twist_keyboard's default output topic is /cmd_vel, which is the
  same topic the OpenCR (hardware) listens to. If we publish there directly:
    - No velocity clamping → can exceed Burger's 0.22 m/s physical limit
    - No acceleration ramp → step changes cause wheel slip + odometry error
    - No safety timeout → robot keeps moving if this terminal crashes

  By remapping to /cmd_vel_raw, all commands pass through velocity_controller
  first, which applies all three safety layers before reaching the hardware.

KEYBOARD CONTROLS (shown at runtime):
  Moving around:
    u  i  o
    j  k  l
    m  ,  .

  i = forward, , = backward, j = turn left, l = turn right, k = stop
  q/z = increase/decrease max speeds by 10%
  w/x = increase/decrease linear speed only
  e/c = increase/decrease angular speed only

USAGE:
  # In Terminal 1 (already running):
  ros2 launch tb3_bringup slam.launch.py fake_joints:=false

  # In Terminal 2:
  ros2 launch tb3_bringup teleop.launch.py

NOTE:
  This launch file uses emulate_tty=True so the node receives keyboard
  input from the terminal where you run `ros2 launch`. You must keep
  focus on this terminal window while driving.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        # emulate_tty=True gives the node a proper terminal (stdin/stdout)
        # so it can read keyboard input when launched via `ros2 launch`.
        emulate_tty=True,
        # Remap the default /cmd_vel output to /cmd_vel_raw so every command
        # passes through velocity_controller's clamp + ramp + timeout.
        remappings=[
            ('cmd_vel', '/cmd_vel_raw'),
        ],
    )

    return LaunchDescription([teleop])
