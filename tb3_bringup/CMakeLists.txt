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

ament_package()
