#!/bin/bash

current_working_dir=$(pwd)

# Function to print messages
function print_message {
    echo -e "\e[32m$1\e[0m"
}

# Function to log errors
function log_error {
    echo -e "\e[31m$1\e[0m" >&2
    exit 1
}

# Function to install necessary dependencies
function install_dependencies {
    print_message "Installing necessary dependencies..."
    sudo apt update
    sudo apt install -y \
        ros-humble-ros2-controllers \
        ros-humble-ur-description \
        ros-humble-xacro \
        python3-pip || log_error "Failed to install necessary dependencies."
    print_message "Installing colcon and vcs tools..."
    sudo apt install -y python3-colcon-common-extensions python3-vcstool || log_error "Failed to install colcon and vcs tools."
    print_message "Installing rosdep..."
    sudo apt install -y python3-rosdep || log_error "Failed to install rosdep."
    # remove rosdep source list if it exists
    sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
    sudo rosdep init || log_error "Failed to initialize rosdep."
    rosdep update || log_error "Failed to update rosdep."
}

# Function to get the git library
function get_git_library {
    local cwd=$(pwd)
    print_message "Cloning the Universal Robots ROS2 Description repository..."
    mkdir -p ./lib/ros2_ws/src
    cd ./lib/ros2_ws/src || exit
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git || log_error "Failed to clone Universal Robots ROS2 Description repository."
    cd "$cwd"
}

# Function to build the ROS2 workspace
function build_ros2_workspace {
    local cwd=$(pwd)
    cd ./lib/ros2_ws || exit
    print_message "Building the ROS2 workspace..."
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release || log_error "Failed to build the ROS2 workspace."
    source install/setup.bash || log_error "Failed to source the workspace."
    cd "$cwd"
}

# Function to convert xacro to urdf
function convert_xacro_to_urdf {
    local xacro_file="./lib/ros2_ws/src/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro"
    local urdf_file="./lib/ros2_ws/src/Universal_Robots_ROS2_Description/urdf/ur3_robot.urdf"
    local name="ur3"

    if [[ -z "$xacro_file" || -z "$urdf_file" || -z "$name" ]]; then
        log_error "Usage: convert_xacro_to_urdf <input.xacro> <output.urdf> <name>"
    fi

    print_message "Sourcing ROS2 and workspace setup files..."
    source /opt/ros/humble/setup.bash
    source ./lib/ros2_ws/install/setup.bash

    print_message "Converting $xacro_file to $urdf_file with name:=$name..."
    xacro "$xacro_file" name:="$name" -o "$urdf_file" || log_error "Failed to convert $xacro_file to $urdf_file"
    print_message "Conversion successful: $urdf_file"
}

# Function to clean up created files and folders
function cleanup {
    print_message "Cleaning up..."
    rm -rf ./tmp ./lib || log_error "Failed to clean up."
}

# Function to display the menu
function display_menu {
    echo "-----------------------------------------------------------------"
    echo "UR3 Setup Script"
    echo "-----------------------------------------------------------------"
    echo "1. Install Dependencies"
    echo "2. Get Git Library"
    echo "3. Build ROS2 Workspace"
    echo "4. Convert Xacro to URDF"
    echo "5. Clean Up"
    echo "6. Exit"
    echo "-----------------------------------------------------------------"
}

# Main script execution
while true; do
    display_menu
    read -p "Choose an option: " choice
    case $choice in
        1)
            install_dependencies
            ;;
        2)
            get_git_library
            ;;
        3)
            build_ros2_workspace
            ;;
        4)
            convert_xacro_to_urdf
            ;;
        5)
            cleanup
            ;;
        6)
            print_message "Exiting..."
            exit 0
            ;;
        *)
            log_error "Invalid option. Please try again."
            ;;
    esac
done
