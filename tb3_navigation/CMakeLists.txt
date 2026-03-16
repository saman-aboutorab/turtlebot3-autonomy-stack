cmake_minimum_required(VERSION 3.8)
project(tb3_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (makes `import tb3_navigation` work)
ament_python_install_package(${PROJECT_NAME})

# Python node scripts
install(PROGRAMS
  # tb3_navigation/nav_goal_sender.py    # added in Step 10
  # tb3_navigation/waypoint_follower.py  # added in Step 11
  DESTINATION lib/${PROJECT_NAME}
)

# C++ nodes (Step 12 — waypoint_follower.cpp port)

# Launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
