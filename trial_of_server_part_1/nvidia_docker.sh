#!/bin/bash

# Define color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Add the NVIDIA Docker GPG key to the system keyring
echo -e "${YELLOW}Adding NVIDIA Docker GPG key...${NC}"
curl -fsSL https://nvidia.github.io/nvidia-docker/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Add the NVIDIA Docker repository to the system sources list
echo -e "${YELLOW}Adding NVIDIA Docker repository...${NC}"
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://nvidia.github.io/nvidia-docker $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/nvidia-docker.list > /dev/null

# Optionally, configure the repository to use experimental packages
echo -e "${YELLOW}Enabling experimental packages in the NVIDIA Docker repository...${NC}"
sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-docker.list

# Update the packages list from the repository
echo -e "${YELLOW}Updating package list...${NC}"
sudo apt-get update

# Install the NVIDIA Container Toolkit
echo -e "${YELLOW}Installing NVIDIA Container Toolkit...${NC}"
sudo apt-get install -y nvidia-container-toolkit

# Restart Docker to apply changes
echo -e "${YELLOW}Restarting Docker...${NC}"
sudo systemctl restart docker

echo -e "${GREEN}NVIDIA Docker installation completed successfully.${NC}"