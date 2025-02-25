cmake_minimum_required(VERSION 3.8)
project(rover_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trusses_custom_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Source files
file(GLOB_RECURSE SRC_FILES src/*.cpp)

# Build the main executable
add_executable(rover_node ${SRC_FILES})

# Link ROS2 and hardware libraries
ament_target_dependencies(rover_node rclcpp std_msgs std_srvs geometry_msgs trusses_custom_interfaces)
#target_link_libraries(turret_node pigpio ncurses) We shoudln't need these ?

# Install the executables
install(TARGETS rover_node
  DESTINATION lib/${PROJECT_NAME})

# Ament package hooks
ament_export_dependencies(rosidl_default_runtime)
ament_package()