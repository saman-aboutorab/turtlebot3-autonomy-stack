cmake_minimum_required(VERSION 3.8)
project(tb3_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files so `ros2 launch tb3_bringup <file>` works
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files so nodes can load YAML params at runtime
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install the hardware/ directory (URDF, assembly notes) into this package's
# share folder. hardware/ has no package.xml so it can't be a standalone ROS2
# package — we host it under tb3_bringup so get_package_share_directory()
# can locate the URDF at runtime from any launch file.
install(DIRECTORY ../hardware
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
