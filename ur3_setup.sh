locale  # check for UTF-8

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common -y
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade -y

sudo apt install ros-humble-desktop ros-humble-gazebo-ros2-control ros-humble-ament-cmake -y


sudo apt install ros-dev-tools -y

# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash

source /opt/ros/humble/setup.bash
# to test the ros2 installation we can run the talker node
# ros2 run demo_nodes_cpp talker

source /opt/ros/humble/setup.bash
# to test the ros2 installation we can run the listener node
# ros2 run demo_nodes_py listener

sudo apt-get remove --purge gazebo*
sudo apt-get autoremove

sudo apt-get update
sudo apt-get install lsb-release gnupg -y


# sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo apt-get update
# sudo apt-get install gz-harmonic

sudo apt-get install gazebo -y

export COLCON_WS=~/workspaces/ur_gazebo
mkdir -p $COLCON_WS/src

cd $COLCON_WS/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
rosdep update && rosdep install --ignore-src --from-paths . -y

cd $COLCON_WS
colcon build --symlink-install

source /opt/ros/humble/setup.bash
source $COLCON_WS/install/setup.bash

# ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py