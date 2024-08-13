#!/bin/bash

# Function to display the menu
show_menu() {
    echo "1) Setup ROS 2 Humble sources"
    echo "2) Install ROS 2 Humble"
    echo "3) Source ROS 2 setup script"
    echo "4) Install additional ROS 2 packages"
    echo "5) View ROS 2 installation details"
    echo "6) Uninstall ROS 2 Humble"
    echo "7) Exit"
}

# Function to setup ROS 2 Humble sources
setup_sources() {
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    echo "ROS 2 Humble sources setup completed."
}

# Function to install ROS 2 Humble
install_ros2() {
    sudo apt update
    sudo apt install ros-humble-desktop -y
    echo "ROS 2 Humble installation completed."
}

# Function to source ROS 2 setup script
source_ros2() {
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    echo "ROS 2 setup script sourced."
}

# Function to install additional ROS 2 packages
install_additional_packages() {
    sudo apt install python3-argcomplete -y
    echo "Additional ROS 2 packages installed."
}

# Function to view ROS 2 installation details
view_ros2_details() {
    dpkg -l | grep ros-humble
}

# Function to uninstall ROS 2 Humble
uninstall_ros2() {
    sudo apt remove --purge ros-humble-* -y
    sudo apt autoremove -y
    sudo rm -rf /opt/ros/humble
    echo "ROS 2 Humble uninstalled."
}

# Main loop
while true; do
    show_menu
    read -p "Enter your choice [1-7]: " choice
    case $choice in
        1) setup_sources ;;
        2) install_ros2 ;;
        3) source_ros2 ;;
        4) install_additional_packages ;;
        5) view_ros2_details ;;
        6) uninstall_ros2 ;;
        7) echo "Exiting..."; exit 0 ;;
        *) echo "Invalid choice, please try again." ;;
    esac
done