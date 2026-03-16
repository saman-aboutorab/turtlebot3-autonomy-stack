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
  # tb3_odometry/odometry_publisher.py  # added in Step 3
  # tb3_odometry/tf2_broadcaster.py     # added in Step 4
  # tb3_odometry/imu_republisher.py     # added in Step 5
  # tb3_odometry/ekf_node.py            # added in Step 6
  # tb3_odometry/velocity_controller.py # added in Step 7
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
