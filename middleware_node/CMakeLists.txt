cmake_minimum_required(VERSION 3.5)
project(middleware_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(middleware_node src/middleware_node.cpp)
ament_target_dependencies(middleware_node rclcpp trajectory_msgs)

install(TARGETS
  middleware_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()