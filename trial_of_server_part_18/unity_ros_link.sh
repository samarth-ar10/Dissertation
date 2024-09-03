#!/bin/bash

# cd ~/workspaces/ur_gazebo/src
# git clone -b main-ros2  https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# source install/setup.bash
# colcon build
# source install/setup.bash

docker build -t foxy -f ros2_docker/Dockerfile .
docker run -it --rm -p 10000:10000 foxy /bin/bash