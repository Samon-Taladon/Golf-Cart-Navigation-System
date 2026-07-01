cmake_minimum_required(VERSION 3.5)
project(navigation_system)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find ament packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install other files (if any)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/websocket_gnss_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
