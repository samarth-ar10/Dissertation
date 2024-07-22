#!/bin/bash

# Function to print the system message
system_message() {
    echo "-------System-------- $1"
}

# Function to start Gazebo
start_gazebo() {
    system_message "Starting Gazebo..."
    export SDL_AUDIODRIVER=dummy
    source /opt/ros/humble/setup.bash
    gazebo
}

# Function to start ROS 2 Node (Example)
start_ros2_node() {
    read -p "Enter the package name: " package_name
    read -p "Enter the node executable name: " node_name
    system_message "Starting ROS 2 node '$node_name' from package '$package_name'..."
    source /opt/ros/humble/setup.bash
    ros2 run $package_name $node_name
}

# Function to launch UR3 robot in Gazebo
launch_ur3_gazebo() {
    system_message "Launching UR3 robot in Gazebo..."
    source /home/dissertation/UR3/ros2_ws/install/setup.bash
    ros2 launch ur_gazebo ur3_gazebo.launch.py
}

# Function to display the menu
display_menu() {
    echo "1. Start Gazebo"
    echo "2. Start ROS 2 Node"
    echo "3. Launch UR3 Robot in Gazebo"
    echo "4. Exit"
}

# Main loop
while true; do
    display_menu
    read -p "Enter your choice [1-4]: " choice
    case $choice in
        1)
            start_gazebo
            ;;
        2)
            start_ros2_node
            ;;
        3)
            launch_ur3_gazebo
            ;;
        4)
            exit 0
            ;;
        *)
            echo "Invalid choice, please try again."
            ;;
    esac
done
