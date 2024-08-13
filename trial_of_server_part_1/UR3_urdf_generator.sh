#!/bin/bash

#get the address of the current dir where the script is running
MAIN_WORKSPACE=$(pwd)

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if ROS 2 is sourced
if ! command_exists ros2; then
    echo "ROS 2 is not sourced. Please source your ROS 2 setup script."
    exit 1
fi

# Install xacro package if not installed
if ! command_exists xacro; then
    echo "Installing xacro package..."
    sudo apt update
    sudo apt install ros-humble-xacro -y
fi

# Install necessary dependencies
echo "Installing necessary dependencies..."
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers -y

# Clone the ur_description repository if it doesn't exist
if [ ! -d "ur_description" ]; then
    echo "Cloning the Universal Robots Description repository..."
    git clone -b rolling https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description
else
    echo "Repository already exists. Skipping clone."
fi

# Navigate to the workspace
echo "Navigating to the workspace..."
cd ur_description

# Build the workspace
echo "Building the workspace..."
colcon build --symlink-install

# Source the workspace
echo "Sourcing the workspace..."
source install/setup.bash

# Navigate to the URDF folder
echo "Navigating to the URDF folder..."
cd urdf || { echo "URDF folder not found!"; exit 1; }

# Convert the ur.urdf.xacro to ur.urdf for user mentioned robot
read -p "Enter the UR robot type (ur3/ur5/ur10) [default: ur3]: " ur_type
ur_type=${ur_type:-ur3}
echo "Converting ur.urdf.xacro to ur.urdf for UR3 robot..."
ros2 run xacro xacro ur.urdf.xacro ur_type:=ur3 name:=${ur_type} > ${ur_type}.urdf

mv ${ur_type}.urdf $MAIN_WORKSPACE/${ur_type}.urdf
rm -rf $MAIN_WORKSPACE/ur_description
echo "Conversion completed. The UR3 URDF file is generated as ur3.urdf."