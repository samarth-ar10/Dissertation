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
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep || log_error "Failed to install necessary dependencies."
    
    # Remove rosdep source list if it exists
    sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
    sudo rosdep init || log_error "Failed to initialize rosdep."
    rosdep update || log_error "Failed to update rosdep."
}

# Function to get the git library
function get_git_library {
    local cwd=$(pwd)
    read -p "Enter the branch you want to use (e.g., humble, main): " branch
    print_message "Cloning the Universal Robots ROS2 Driver repository ($branch branch)..."
    mkdir -p ./lib/ros2_ws/src
    cd ./lib/ros2_ws || exit
    git clone -b "$branch" https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver || log_error "Failed to clone Universal Robots ROS2 Driver repository."
    vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.humble.repos || log_error "Failed to import repositories."
    
    print_message "Installing dependencies using rosdep..."
    rosdep update
    rosdep install --ignore-src --from-paths src -y || log_error "Failed to install dependencies."
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

# Function to clean up created files and folders
function cleanup {
    print_message "Cleaning up..."
    rm -rf ./tmp ./lib || log_error "Failed to clean up."
}

# Function to display the menu
function display_menu {
    echo "-----------------------------------------------------------------"
    echo "1. Install Dependencies"
    echo "2. Get Git Library"
    echo "3. Build ROS2 Workspace"
    echo "4. Clean Up"
    echo "5. Exit"
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
            cleanup
            ;;
        5)
            print_message "Exiting..."
            exit 0
            ;;
        *)
            log_error "Invalid option. Please try again."
            ;;
    esac
done
