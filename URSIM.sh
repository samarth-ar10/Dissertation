#!/bin/bash

# Update the system
sudo apt update && sudo apt upgrade -y
ROS_DISTRO=rolling

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Install necessary packages
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y ros-dev-tools

sudo apt update

sudo apt upgrade -y

sudo apt install -y ros-rolling-desktop ros-rolling-ur-client-library

# Source the ROS 2 setup file
source /opt/ros/rolling/setup.bash

# Install ur package
sudo apt-get install -y ros-${ROS_DISTRO}-ur

# Run the ursim.sh script using ros2 run with a different port
docker run -d --name ursim --network ursim_net -p 30000:29999 -e ROBOT_MODEL=UR3 universalrobots/ursim_e-series