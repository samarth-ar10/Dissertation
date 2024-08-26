#!/bin/bash

# Function to print messages
print_message() {
    echo "-----------------------------------------------------------------"
    echo "$1"
    echo "-----------------------------------------------------------------"
}

# Function to launch UR3 in Gazebo
launch_gazebo() {
    print_connection_details
    print_message "Launching UR3 in Gazebo..."
    source /opt/ros/rolling/setup.bash
    source /root/workspaces/ur_gazebo/install/setup.bash
    # ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
    ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur3 
}

# Function to connect to the real UR3 robot
connect_real_robot() {
    print_connection_details
    print_message "Connecting to the real UR3 robot..."
    source /opt/ros/rolling/setup.bash
    source /root/workspaces/ur_gazebo/install/setup.bash
    ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=$ROBOT_IP ur_type:=ur3
}

# Function to print connection details
print_connection_details() {
    print_message "Connection Details"
    echo "Container IP: $(hostname -I | awk '{print $1}')"
    echo "Gazebo Master URI: http://$(hostname -I | awk '{print $1}'):11345"
    echo "ROS 2 Domain ID: $ROS_DOMAIN_ID"
    echo "To control UR3 using ROS 2 commands, use the following:"
    echo "  ros2 topic pub /ur3/joint_trajectory_controller/joint_trajectory <message>"
}

# Main script execution
print_message "UR3 Launch Script"
echo "1. Launch UR3 in Gazebo"
echo "2. Connect to the real UR3 robot"
echo "3. Print Connection Details"
read -p "Choose an option: " choice

case $choice in
    1)
        launch_gazebo
        ;;
    2)
        read -p "Enter the IP address of the real UR3 robot: " ROBOT_IP
        connect_real_robot
        ;;
    3)
        print_connection_details
        ;;
    *)
        echo "Invalid option. Please try again."
        ;;
esac