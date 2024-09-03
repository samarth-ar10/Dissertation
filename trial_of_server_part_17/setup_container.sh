#!/bin/bash

# Source ROS 2 setup files
source /opt/ros/humble/setup.bash
source /root/workspaces/ur_gz/install/setup.bash

# Set the Gazebo Master URI
export GAZEBO_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11345

# Start the Gazebo server
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py