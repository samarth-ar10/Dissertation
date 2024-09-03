cmake_minimum_required(VERSION 3.5)
project(middle_man)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(middle_man_node src/middle_man_node.cpp)
ament_target_dependencies(middle_man_node rclcpp trajectory_msgs)

install(TARGETS
  middle_man_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()