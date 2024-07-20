#!/bin/bash

# Project Information
# This script is developed as part of a dissertation project by Samarth
# under the University of Nottingham.

# Systems this script has been tested on:
# - Ubuntu 22.04 (Jammy Jellyfish)

# Users running on different systems or configurations can add their notes below:
# - System 1: (Add your system details here)
# - System 2: (Add your system details here)

# Initialize the container sequence number
CONTAINER_SEQ_FILE=".container_seq"
if [ ! -f $CONTAINER_SEQ_FILE ]; then
    echo 0 > $CONTAINER_SEQ_FILE
fi

# Function to print the system message
system_message() {
    echo "-------System-------- $1"
}

# Function to check if Docker is installed
check_docker_installed() {
    system_message "Checking if Docker is installed..."
    if ! command -v docker &> /dev/null; then
        echo "Docker is not installed."
        return 1
    else
        echo "Docker is installed."
        return 0
    fi
}

# Function to install Docker
install_docker() {
    system_message "Installing Docker..."
    sudo apt-get update
    sudo apt-get install -y apt-transport-https ca-certificates curl software-properties-common
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
    sudo apt-get update

    read -p "Enter Docker version to install (or press Enter for default): " docker_version
    if [ -z "$docker_version" ]; then
        sudo apt-get install -y docker-ce docker-ce-cli containerd.io
    else
        sudo apt-get install -y docker-ce=$docker_version docker-ce-cli=$docker_version containerd.io
    fi

    sudo usermod -aG docker $USER
    system_message "Docker installation completed. You may need to log out and log back in for group changes to take effect."
}

# Function to remove Docker
remove_docker() {
    system_message "Removing Docker..."
    sudo apt-get purge -y docker-ce docker-ce-cli containerd.io
    sudo apt-get autoremove -y --purge docker-ce docker-ce-cli containerd.io
    sudo rm -rf /var/lib/docker
    sudo rm -rf /var/lib/containerd
    # Remove Docker installed via snap
    sudo snap remove docker
    system_message "Docker has been removed from the system."
}

# Function to start Docker daemon
start_docker_daemon() {
    system_message "Starting Docker daemon..."
    if ! systemctl is-active --quiet docker; then
        if sudo systemctl start docker && sudo systemctl enable docker; then
            echo "Docker daemon started."
        else
            echo "Failed to start Docker daemon. Please ensure Docker is installed correctly."
            exit 1
        fi
    else
        echo "Docker daemon is already running."
    fi
}

# Function to check Docker daemon status
check_docker_daemon() {
    system_message "Checking Docker daemon status..."
    if ! systemctl is-active --quiet docker; then
        echo "Docker daemon is not running. Please start Docker and try again."
        exit 1
    fi
}

# Function to check Docker permissions
check_docker_permissions() {
    system_message "Checking Docker permissions..."
    if ! docker info > /dev/null 2>&1; then
        echo "ERROR: Permission denied while trying to connect to the Docker daemon socket."
        echo "Please ensure you have the necessary permissions to use Docker."
        echo "You might need to add your user to the 'docker' group and re-login:"
        echo "    sudo usermod -aG docker \$USER"
        echo "    newgrp docker"
        exit 1
    fi
}

# Function to build the Docker image with retries
build_docker_image() {
    system_message "Building the Docker image..."
    local retries=5
    local count=0
    until docker build -t ros2_gazebo .; do
        count=$((count + 1))
        if [ $count -lt $retries ]; then
            echo "Retrying to build the Docker image ($count/$retries)..."
            sleep 5
        else
            echo "Failed to build Docker image after $retries attempts. Please check the Dockerfile for errors."
            return 1
        fi
    done
    echo "Docker image built successfully."
}

# Function to run the Docker container
run_docker_container() {
    read -p "Enter a name for the Docker container: " container_name

    # Check if the container already exists
    if docker ps -a --format '{{.Names}}' | grep -q "^${container_name}$"; then
        system_message "Docker container '$container_name' already exists. Starting the container..."
        if docker start "$container_name"; then
            echo "Docker container '$container_name' started successfully."
            enter_running_docker_container "$container_name"
        else
            echo "Failed to start Docker container '$container_name'."
        fi
    else
        system_message "Creating and running a new Docker container '$container_name'..."
        # Check if the user wants to create a local storage
        read -p "Do you want to create a local storage for the container? (y/n): " storage_choice
        if [ "$storage_choice" == "y" ]; then
            local storage_path="$(pwd)/storage/$container_name"
            mkdir -p "$storage_path"
            echo "Local storage created at '$storage_path'."

            # Run the container with local storage
            if docker run -d --name "$container_name" \
                           --mount type=bind,source="$storage_path",target=/home/dissertation \
                           ros2_gazebo /bin/bash -c "while :; do sleep 1; done"; then
                echo "Docker container '$container_name' is running with persistence."
                enter_running_docker_container "$container_name"
            else
                echo "Failed to run Docker container. Please check if the image was built successfully."
            fi
        else
            # Run the container without local storage
            if docker run -d --name $container_name -e SDL_AUDIODRIVER=dummy -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY ros2_gazebo ; then
                enter_running_docker_container "$container_name"
            else
                echo "Failed to run Docker container. Please check if the image was built successfully."
            fi
        fi
    fi
}

# Function to stop the Docker container
stop_docker_container() {
    read -p "Enter the name of the Docker container to stop (or '*' to stop all containers): " container_name
    if [ "$container_name" == "*" ]; then
        system_message "Stopping all Docker containers..."
        if docker stop $(docker ps -q); then
            echo "All Docker containers stopped successfully."
        else
            echo "Failed to stop all Docker containers."
        fi
    else
        system_message "Stopping the Docker container '$container_name'..."
        if docker stop "$container_name"; then
            echo "Docker container '$container_name' stopped successfully."
        else
            echo "Failed to stop Docker container '$container_name'. It might not be running."
        fi
    fi
}

# Function to remove the Docker container
remove_docker_container() {
    read -p "Enter the name of the Docker container to remove (or '*' to remove all containers): " container_name
    if [ "$container_name" == "*" ]; then
        system_message "Removing all Docker containers..."
        if docker rm $(docker ps -a -q); then
            echo "All Docker containers removed successfully."
        else
            echo "Failed to remove all Docker containers."
        fi
    else
        system_message "Removing the Docker container '$container_name'..."
        if docker rm "$container_name"; then
            echo "Docker container '$container_name' removed successfully."
        else
            echo "Failed to remove Docker container '$container_name'. It might not exist."
        fi
    fi
}

# Function to list all Docker containers
list_all_docker_containers() {
    system_message "Listing all Docker containers..."
    docker ps -a --format "table {{.ID}}\t{{.Names}}\t{{.Status}}"
}

# Function to enter a running Docker container
enter_running_docker_container() {
    local container_name="$1"
    system_message "Entering the Docker container '$container_name'..."
    if docker exec -it "$container_name" /bin/bash; then
        echo "Entered Docker container '$container_name'."
    else
        echo "Failed to enter Docker container '$container_name'. Ensure it is running."
    fi
}

#

# Function to display the menu
display_menu() {
    echo "------------------------------------Docker Menu------------------------------------"
    echo "1. Install Docker"
    echo "2. Start Docker Daemon"
    echo "3. Build Docker Image"
    echo "4. Run Docker Container"
    echo "5. Stop Docker Container"
    echo "6. Remove Docker Container"
    echo "7. List All Docker Containers"
    echo "8. Enter Running Docker Container"
    echo "9. Remove Docker"
    echo "10. Exit"
    echo "-----------------------------------------------------------------------------------"
}

# Main loop
while true; do
    display_menu
    read -p "Enter your choice: " choice
    echo "-----------------------------------------------------------------------------------"
    case $choice in
        1)
            check_docker_installed || install_docker
            ;;
        2)
            start_docker_daemon
            ;;
        3)
            check_docker_daemon && check_docker_permissions && build_docker_image
            ;;
        4)
            check_docker_daemon && run_docker_container
            ;;
        5)
            check_docker_daemon && stop_docker_container
            ;;
        6)
            check_docker_daemon && remove_docker_container
            ;;
        7)
            check_docker_daemon && list_all_docker_containers
            ;;
        8)
            check_docker_daemon && read -p "Enter the name of the Docker container to enter: " container_name && enter_running_docker_container "$container_name"
            ;;
        9)
            remove_docker
            ;;
        10)
            break
            ;;
        *)
            echo "Invalid choice. Please enter a valid option."
            ;;
    esac
done
