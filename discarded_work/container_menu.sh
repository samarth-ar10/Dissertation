#!/bin/bash

# Function to print messages
print_message() {
    echo "-----------------------------------------------------------------"
    echo "$1"
    echo "-----------------------------------------------------------------"
}

# Function to start Gazebo server
start_gazebo_server() {
    print_message "Starting Gazebo server..."
    source /root/install/setup.bash
    source /opt/ros/humble/setup.bash
    source /root/workspaces/ur_gz/install/setup.bash
    ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
}

# Function to start Gazebo client
start_gazebo_client() {
    print_message "Starting Gazebo client..."
    gzclient
}

# Function to spawn UR3 in Gazebo
spawn_ur3_in_gazebo() {
    print_message "Spawning UR3 in Gazebo..."
    source /opt/ros/humble/setup.bash
    source /root/workspaces/ur_gz/install/setup.bash
    ros2 launch ur_description ur3_spawn.launch.py
}

# Function to display the menu
display_menu() {
    echo "-----------------------------------------------------------------"
    echo "1. Start Gazebo Server"
    echo "2. Start Gazebo Client"
    echo "3. Spawn UR3 in Gazebo"
    echo "4. Exit"
    echo "-----------------------------------------------------------------"
}

# Main script execution
while true; do
    display_menu
    read -p "Choose an option: " choice
    case $choice in
        1)
            start_gazebo_server
            ;;
        2)
            start_gazebo_client
            ;;
        3)
            spawn_ur3_in_gazebo
            ;;
        4)
            print_message "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid option. Please try again."
            ;;
    esac
done
