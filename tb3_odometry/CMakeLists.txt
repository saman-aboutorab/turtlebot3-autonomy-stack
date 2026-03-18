cmake_minimum_required(VERSION 3.8)
project(tb3_odometry)

# Compiler warnings (good practice)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# --- Python package install ---
# This makes `import tb3_odometry` work in any Python node in this package.
# ament_cmake_python installs the tb3_odometry/ subdirectory as a Python package.
ament_python_install_package(${PROJECT_NAME})

# --- Python node scripts ---
# Each entry here becomes a `ros2 run tb3_odometry <script_name>` command.
# We register scripts here as we add them in later steps.
install(PROGRAMS
  tb3_odometry/tf2_checker.py           # Step 2 — verify URDF TF tree
  tb3_odometry/odometry_publisher.py   # Step 3 — wheel encoder → /odom
  tb3_odometry/tf2_broadcaster.py      # Step 4 — /odom → TF odom→base_footprint
  tb3_odometry/imu_republisher.py      # Step 5 — raw /imu → clean /imu/data
  tb3_odometry/ekf_node.py             # Step 6 — EKF: /odom + /imu/data → /odometry/filtered
  tb3_odometry/velocity_controller.py  # Step 7 — /cmd_vel_raw → clamp/ramp/timeout → /cmd_vel
  DESTINATION lib/${PROJECT_NAME}
)

# --- C++ nodes (Step 12) ---
# C++ source files and their target_link_libraries will be added here
# when we port the Python nodes to C++ in Step 12.

# --- Config and launch files ---
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL  # OK if config/ is empty during early steps
)

ament_package()
