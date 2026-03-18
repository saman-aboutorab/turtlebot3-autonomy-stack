"""
sensors.launch.py
=================

Starts the two sensor-processing nodes that sit directly on top of the raw
hardware topics published by the OpenCR firmware.

  OpenCR publishes:
    /joint_states  (sensor_msgs/JointState)  — wheel encoder positions
    /imu           (sensor_msgs/Imu)          — raw IMU, no covariances

  This launch file processes those into clean, standard topics:
    /odom          (nav_msgs/Odometry)         — pose + velocity from wheels
    /imu/data      (sensor_msgs/Imu)           — IMU with proper covariances

WHY A SEPARATE FILE:
  sensors.launch.py is included by robot.launch.py, which adds the EKF, TF
  broadcaster, and velocity controller on top. Having it separate lets you
  run just the sensor layer in isolation for debugging or on a different
  machine (e.g., everything on the RPi4, nothing on the laptop).

USAGE:
  ros2 launch tb3_bringup sensors.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    odometry_publisher = Node(
        package='tb3_odometry',
        executable='odometry_publisher.py',
        name='odometry_publisher',
        output='screen',
        # Parameters can be overridden here if needed, e.g.:
        # parameters=[{'wheel_radius': 0.033, 'wheel_separation': 0.160}]
    )

    imu_republisher = Node(
        package='tb3_odometry',
        executable='imu_republisher.py',
        name='imu_republisher',
        output='screen',
    )

    return LaunchDescription([
        odometry_publisher,
        imu_republisher,
    ])
