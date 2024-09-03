cd $COLCON_WS/src
ros2 pkg create --build-type ament_cmake middleware_node --dependencies rclcpp std_msgs ur_robot_driver ur_simulation_gazebo

cd $COLCON_WS/src/middleware_node/src
touch middleware_node.cpp


