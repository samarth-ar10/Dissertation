#!/bin/bash

# Function to print messages
function print_message {
    echo "-----------------------------------------------------------------"
    echo "$1"
    echo "-----------------------------------------------------------------"
}

# Function to log errors
function log_error {
    echo "ERROR: $1" >> "$ERROR_LOG"
}

# Function to install and setup ROS2 and Gazebo
function install_and_setup {
    print_message "Updating and upgrading the system..."
    sudo apt update && sudo apt upgrade -y || log_error "Failed to update and upgrade the system."

    print_message "Installing necessary dependencies..."
    sudo apt install -y \
        curl \
        gnupg2 \
        lsb-release \
        build-essential \
        cmake \
        git \
        libboost-all-dev \
        libssl-dev \
        pkg-config \
        python3 \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        wget \
        software-properties-common \
        alsa-utils || log_error "Failed to install necessary dependencies."

    print_message "Setting up the locale..."
    sudo locale-gen en_US en_US.UTF-8 && \
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 || log_error "Failed to set up the locale."

    print_message "Adding the ROS 2 GPG key and repository..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' || log_error "Failed to add the ROS 2 GPG key and repository."

    print_message "Installing ROS 2 Humble..."
    sudo apt update && sudo apt install -y ros-humble-desktop || log_error "Failed to install ROS 2 Humble."

    print_message "Sourcing ROS 2 setup script..."
    echo "source /opt/ros/humble/setup.bash" >> /etc/skel/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc || log_error "Failed to source ROS 2 setup script."

    print_message "Installing colcon..."
    pip3 install -U colcon-common-extensions || log_error "Failed to install colcon."

    print_message "Initializing rosdep..."
    sudo rosdep init && rosdep update || log_error "Failed to initialize rosdep."

    print_message "Adding the Gazebo GPG key and repository..."
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && sudo apt update || log_error "Failed to add the Gazebo GPG key and repository."

    print_message "Installing Gazebo and ros_gz..."
    sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs || log_error "Failed to install Gazebo and ros_gz."

    print_message "Creating directories for organization..."
    mkdir -p ~/lib ~/log ~/user ~/lib/ros2_ws/src || log_error "Failed to create directories for organization."

    print_message "Installation and setup completed successfully!"
}

# Function to uninstall ROS2 and Gazebo
function uninstall {
    print_message "Uninstalling ROS2 and Gazebo..."
    sudo apt remove --purge -y ros-humble-* gazebo* colcon* && sudo apt autoremove -y || log_error "Failed to remove ROS2, Gazebo, and colcon."
    sudo rm -rf ~/lib/ros2_ws || log_error "Failed to remove ~/lib/ros2_ws."
    print_message "Uninstallation completed!"
}

# Function to run the setup
function run_setup {
    print_message "Running the setup..."
    source /opt/ros/humble/setup.bash || log_error "Failed to source ROS 2 setup script."
    source ~/lib/ros2_ws/install/setup.bash || log_error "Failed to source the workspace setup script."
    # export SDL_AUDIODRIVER=dummy
    gazebo || log_error "Failed to start Gazebo."
    print_message "Setup completed!"
}

# Function to display the menu
function display_menu {
    echo "-----------------------------------------------------------------"
    echo "ROS2 and Gazebo Setup Script"
    echo "-----------------------------------------------------------------"
    echo "1. Install and Setup"
    echo "2. Uninstall"
    echo "3. Run Setup"
    echo "4. Exit"
    echo "-----------------------------------------------------------------"
}

# Main script logic
ERROR_LOG="./log/error_log.txt"

# Function to parse arguments and run commands
function parse_arguments {
    while getopts ":isu" opt; do
        case $opt in
            i)
                install_and_setup
                ;;
            s)
                run_setup
                ;;
            u)
                uninstall
                ;;
            \?)
                echo "Invalid option: -$OPTARG" >&2
                exit 1
                ;;
        esac
    done
}

if [ $# -gt 0 ]; then
    parse_arguments "$@"
else
    while true; do
        display_menu
        read -p "Enter your choice: " choice
        echo "-----------------------------------------------------------------------------------"
        case $choice in
            1)
                install_and_setup
                ;;
            2)
                uninstall
                ;;
            3)
                run_setup
                ;;
            4)
                exit 0
                ;;
            *)
                echo "Invalid choice. Please enter a valid option."
                ;;
        esac
    done
fi
