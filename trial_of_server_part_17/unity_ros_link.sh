#!/bin/bash

cd ~/workspaces/ur_gazebo/src
git clone -b main-ros2  https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

source install/setup.bash
colcon build
source install/setup.bash