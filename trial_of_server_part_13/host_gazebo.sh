#!/bin/bash

# Set the address of the master URI to the container's public address
read -p "Enter the IP address of the container (default: 172.17.0.2): " container_ip
container_ip=${container_ip:-172.17.0.2}
read -p "Enter the port of the container (default: 11345): " container_port
container_port=${container_port:-11345}
export GAZEBO_MASTER_URI=http://$container_ip:$container_port
echo "Connecting to IP: $container_ip and port: $container_port"

# Connect to the Gazebo server inside the container using gzclient
gzclient