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
  tb3_navigation/nav_goal_sender.py      # Step 10 — single goal Nav2 client
  tb3_navigation/waypoint_follower.py   # Step 11 — multi-waypoint mission
  DESTINATION lib/${PROJECT_NAME}
)

# C++ nodes (Step 12 — waypoint_follower.cpp port)

# Launch, config, and map files
# maps/ is at the repo root — installed here so navigation.launch.py can
# reference them via get_package_share_directory('tb3_navigation').
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ../maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
