: '
MIT License

Author: Samarth
Contact: psxs2@nottingham.ac.uk

This code is prepared for the dissertation project at the University of Nottingham.
The University of Nottingham has governance over this code.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'

: '
This script, `docker_menu.sh`, provides a command-line menu interface for managing Docker containers. 
It allows users to build Docker images, run containers, list all containers, stop containers, delete 
containers, and enter running containers.

Detailed Explanation:
1. Display Menu: The `display_menu` function prints a menu with options for different Docker operations. 
   It helps users navigate through the available commands.
   - Function:
     ```sh
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
     ```

2. Build Docker Image: This function prompts the user for the Dockerfile path and image name, then builds 
   the Docker image using the provided information.
   - Function:
     ```sh
     build_image() {
         read -p "Enter the path to the Dockerfile: " dockerfile_path
         read -p "Enter the name for the Docker image: " image_name
         docker build -t $image_name -f $dockerfile_path .
     }
     ```

3. Run Docker Container: This function prompts the user for the image name and container name, then runs 
   the Docker container using the provided information.
   - Function:
     ```sh
     run_container() {
         read -p "Enter the Docker image name: " image_name
         read -p "Enter the name for the Docker container: " container_name
         docker run -d --name $container_name $image_name
     }
     ```

4. List All Docker Containers: This function lists all Docker containers, including their status and IDs.
   - Function:
     ```sh
     list_containers() {
         docker ps -a
     }
     ```

5. Stop Docker Container: This function prompts the user for the container name and stops the specified 
   Docker container.
   - Function:
     ```sh
     stop_container() {
         read -p "Enter the name of the Docker container to stop: " container_name
         docker stop $container_name
     }
     ```

6. Delete Docker Container: This function prompts the user for the container name and deletes the specified 
   Docker container.
   - Function:
     ```sh
     delete_container() {
         read -p "Enter the name of the Docker container to delete: " container_name
         docker rm $container_name
     }
     ```

7. Enter Running Docker Container: This function prompts the user for the container name and opens an 
   interactive shell inside the specified running Docker container.
   - Function:
     ```sh
     enter_container() {
         read -p "Enter the name of the running Docker container to enter: " container_name
         docker exec -it $container_name /bin/bash
     }
     ```

How to Invoke the Script:
To invoke the script, you can run it from the command line. For example:
```sh
./docker_menu.sh
```
'

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

# # Function to run the Docker container
# run_container() {
#     read -p "Enter the name for the Docker container: " container_name
#     read -p "Enter the gazebo port to expose on the host machine (default: 11345): " gz_port
#     read -p "Enter the ROS 2 port to expose on the host machine (default: 11311): " ros_port
#     read -p "Enter the ROS2-TCP endpoint (default: 10000): " ros2_tcp_endpoint_
#     gz_port=${gz_port:-11345}
#     ros_port=${ros_port:-11311}
#     ros2_tcp_endpoint_=${ros2_tcp_endpoint_:-10000}
    
#     sudo ip link add macvlan-br0 link wlp0s20f3 type macvlan mode bridge
#     sudo ip addr add 192.168.1.100/24 dev macvlan-br0
#     sudo ip link set macvlan-br0 up

#     sudo sysctl -w net.ipv4.ip_forward=1
#     sudo ip route add 192.168.1.0/24 dev macvlan-br0sudo ip route add 192.168.1.0/24 dev macvlan-br0


#    # Check if the network exists before creating it
#     if ! docker network inspect macvlan_net >/dev/null 2>&1; then
#         docker network create -d macvlan \
#             --subnet=192.168.1.0/24 \
#             --gateway=192.168.1.1 \
#             -o parent=wlp0s20f3 \
#             macvlan_net
#     fi

#     # Allow X11 forwarding if not already allowed
#     xhost +local:root || true

#     # Run the Docker container with X11 forwarding
#     docker run -it --name "$container_name" \
#         -p "$gz_port":11345 \
#         -p "$ros_port":11311 \
#         -p "$ros2_tcp_endpoint_":10000 \
#         --network macvlan_net \
#         -e DISPLAY=$DISPLAY \
#         -v /tmp/.X11-unix:/tmp/.X11-unix \
#         my_ros2_image
# }

# Function to run the Docker container
run_container() {
    read -p "Enter the name for the Docker container: " container_name
    read -p "Enter the gazebo port to expose on the host machine (default: 11345): " gz_port
    read -p "Enter the ROS 2 port to expose on the host machine (default: 11311): " ros_port
    read -p "Enter the ROS-TCP Endpoint port to expose on the host machine (default: 10000): " ros_tcp_port
    
    gz_port=${gz_port:-11345}
    ros_port=${ros_port:-11311}
    ros_tcp_port=${ros_tcp_port:-10000}
    
    # Allow X11 forwarding if not already allowed
    xhost +local:root || true

    # Run the container and map the required ports
    docker run -it --name "$container_name" \
    --network host \
    -e GAZEBO_MASTER_URI=http://localhost:"$gz_port" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -p "$gz_port:$gz_port" \
    -p "$ros_port:$ros_port" \
    -p "$ros_tcp_port:10000" \
    my_ros2_image
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