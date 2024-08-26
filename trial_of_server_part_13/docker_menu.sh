#!/bin/bash

# Function to display the menu
display_menu() {
    echo "------------------------------------Docker Menu------------------------------------"
    echo "1. Build Docker Image"
    echo "2. Run Docker Container"
    echo "3. List All Docker Containers"
    echo "4. Stop Docker Container"
    echo "5. Delete Docker Container"
    echo "6. Enter Running Docker Container"
    echo "7. Exit"
    echo "-----------------------------------------------------------------------------------"
}

# Function to build the Docker image
build_image() {
    docker build -t my_ros2_image .
}

# Function to run the Docker container
run_container() {
    read -p "Enter the name for the Docker container: " container_name
    read -p "Enter the gazebo port to expose on the host machine (default: 11345): " gz_port
    read -p "Enter the ROS 2 port to expose on the host machine (default: 11311): " ros_port
    gz_port=${gz_port:-11345}
    ros_port=${ros_port:-11311}
    
    docker run -it --name "$container_name" --network host -e GAZEBO_MASTER_URI=http://localhost:"$gz_port" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix my_ros2_image
}

# Function to list all Docker containers
list_containers() {
    docker ps -a --format "table {{.ID}}\t{{.Names}}\t{{.Status}}"
}

# Function to stop the Docker container
stop_container() {
    read -p "Enter the name of the Docker container to stop: " container_name
    docker stop "$container_name"
}

# Function to delete the Docker container
delete_container() {
    read -p "Enter the name of the Docker container to delete: " container_name
    docker rm "$container_name"
}

# Function to enter a Docker container, starting it if necessary
enter_running_docker_container() {
    read -p "Enter the name or ID of the Docker container to enter: " container_name
    if docker ps -a --format '{{.Names}}' | grep -q "^${container_name}$"; then
        if ! docker ps --format '{{.Names}}' | grep -q "^${container_name}$"; then
            echo "Container '$container_name' is not running. Starting it..."
            docker start "$container_name"
        fi
        docker exec -it "$container_name" /bin/bash
        echo "Entered Docker container '$container_name'."
    else
        echo "Container '$container_name' does not exist."
    fi
}

# Main menu loop
while true; do
    display_menu
    read -p "Enter your choice [1-7]: " choice
    echo "-----------------------------------------------------------------------------------"
    case $choice in
        1)
            build_image
            ;;
        2)
            run_container
            ;;
        3)
            list_containers
            ;;
        4)
            stop_container
            ;;
        5)
            delete_container
            ;;
        6)
            enter_running_docker_container
            ;;
        7)
            break
            ;;
        *)
            echo "Invalid choice. Please enter a valid option."
            ;;
    esac
done