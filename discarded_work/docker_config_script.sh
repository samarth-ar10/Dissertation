#!/bin/bash
system_message_flag=1 #set to 1 to enable system messages
container_level_message_flag=0 # set to 1 to enable container level messages
# Ensure the container name is passed as an argument
if [ -z "$1" ]; then
  echo "Usage: $0 <container_name>"
  exit 1
fi

# Function to print the system message
system_message() {
    if [ $system_message_flag -eq 1 ]; then
        echo "-------System-------- $1"
    fi
}

container_level_message() {
    if [ $container_level_message_flag -eq 1 ]; then
        echo "-------Container------ $1"
    fi
}
container_name=$1

# Create a new user
system_message "Creating a new user..."
if docker exec "$container_name" useradd -m -s /bin/bash dissertation 
then
    container_level_message "User created successfully."
else
    container_level_message "User already exists."
fi
if docker exec "$container_name" usermod -aG sudo dissertation
then
    container_level_message "User added to the sudo group."
else
    container_level_message "User already in the sudo group."
fi
if docker exec "$container_name" bash -c 'echo "dissertation:dissertation" | chpasswd'
then
    container_level_message "User password set successfully."
else
    container_level_message "User password already set."
fi

# Set the default user and working directory
system_message "Setting the default user and working directory..."
if docker exec "$container_name" echo 'export USER=dissertation' >> /etc/profile
then
    container_level_message "Default user set successfully."
else
    container_level_message "Default user already set."
fi
if docker exec "$container_name" echo 'cd /home/dissertation' >> /etc/profile
then
    container_level_message "Working directory set successfully."
else
    container_level_message "Working directory already set."
fi

# wait for the user to press enter if -y flag is not set
if [ "$2" != "-y" ]; then
    read -p "Press enter to continue..."
fi

# Copy necessary files to the container
system_message "Copying necessary files to the container..."
if docker exec "$container_name" mkdir -p /home/dissertation/UR3   
then
    container_level_message "Directory UR3 created successfully."
else
    container_level_message "Directory UR3 already exists."
fi
if docker cp container_menu.sh "$container_name:/home/dissertation/UR3"
then
    container_level_message "File container_menu.sh copied successfully."
else
    container_level_message "File container_menu.sh already exists."
fi
if docker exec "$container_name" chmod +x /home/dissertation/UR3/container_menu.sh
then
    container_level_message "File permissions for container_menu.sh set successfully."
else
    container_level_message "File permissions for container_menu.sh already set."
fi
if docker exec "$container_name" mkdir -p /home/dissertation/UR3/launch
then
    container_level_message "Directory UR3/launch created successfully."
else
    container_level_message "Directory UR3/launch already exists."
fi
if docker cp ur3_bringup.launch.py "$container_name:/home/dissertation/UR3/launch"
then
    container_level_message "File copied ur3_bringup.launch.py successfully."
else
    container_level_message "File ur3_bringup.launch.py already exists."
fi
if docker exec "$container_name" mkdir -p /home/dissertation/UR3/ros2_ws/src
then
    container_level_message "Directory ros2_ws/src created successfully."
else
    container_level_message "Directory ros2_ws/src already exists."
fi
#
# wait for the user to press enter if -y flag is not set
if [ "$2" != "-y" ]; then
    read -p "Press enter to continue..."
fi

# Update and install necessary tools
system_message "Updating and installing necessary tools..."
if docker exec -it "$container_name" bash -c "
    apt-get update && \
    apt-get install -y \
        curl \
        gnupg \
        lsb-release \
        tzdata \
        apt-transport-https \
        ca-certificates \
        software-properties-common \
        sudo \
        xvfb \
        xfce4 \
        mesa-utils \
        libgl1-mesa-glx \
        alsa-utils \
        pulseaudio \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*
"
then
    container_level_message "Tools installed successfully."
else
    container_level_message "Tools already installed."
fi

# wait for the user to press enter if -y flag is not set
if [ "$2" != "-y" ]; then
    read -p "Press enter to continue..."
fi

# Add ROS 2 GPG key and repository
system_message "Adding ROS 2 GPG key and repository..."
if docker exec -it "$container_name" bash -c "
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo 'deb http://packages.ros.org/ros2/ubuntu \$(lsb_release -cs) main' | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update
"
then
    system_message "ROS 2 GPG key and repository added successfully."
else
    system_message "ROS 2 GPG key and repository already added."
fi

# Install ROS 2 packages
system_message "Installing ROS 2 packages..."
if docker exec -it "$container_name" bash -c "
    apt-get install -y \
        ros-humble-desktop \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-ur-msgs \
        ros-humble-ur-dashboard-msgs \
        alsa-utils \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        git \
    && rm -rf /var/lib/apt/lists/*
"
then
    system_message "ROS 2 packages installed successfully."
else
    system_message "ROS 2 packages already installed."
fi

# Install colcon
system_message "Installing colcon..."
if docker exec -it "$container_name" python3 -m pip install -U colcon-common-extensions
then 
    system_message "Colcon installed successfully."
else
    system_message "Colcon already installed."
fi

# Source the ROS 2 setup script
system_message "Sourcing the ROS 2 setup script..."
if docker exec -it "$container_name" bash -c "
    echo 'source /opt/ros/humble/setup.bash' >> /etc/skel/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
"
then
    system_message "ROS 2 setup script sourced successfully."
else
    system_message "ROS 2 setup script already sourced."
fi

# Initialize rosdep
system_message "Initializing rosdep..."
if docker exec -it "$container_name" bash -c "
    rosdep init && \
    rosdep update
"
then
    system_message "Rosdep initialized successfully."
else
    system_message "Rosdep already initialized."
fi

# wait for the user to press enter if -y flag is not set
if [ "$2" != "-y" ]; then
    read -p "Press enter to continue..."
fi

# Add Gazebo GPG key and repository
system_message "Adding Gazebo GPG key and repository..."
if docker exec -it "$container_name" bash -c "
    curl -sSL http://packages.osrfoundation.org/gazebo.key | apt-key add - && \
    echo 'deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \$(lsb_release -cs) main' | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update
"
then
    system_message "Gazebo GPG key and repository added successfully."
else
    system_message "Gazebo GPG key and repository already added."
fi

# Install Gazebo and related packages
system_message "Installing Gazebo and related packages..."
if docker exec -it "$container_name" bash -c "
    apt-get install -y \
        gazebo \
        ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*
"
then
    system_message "Gazebo and related packages installed successfully."
else
    system_message "Gazebo and related packages already installed."
fi

# wait for the user to press enter if -y flag is not set
if [ "$2" != "-y" ]; then
    read -p "Press enter to continue..."
fi

# Install Universal Robots ROS 2 packages
system_message "Installing Universal Robots ROS 2 packages..."
if docker exec -it "$container_name" bash -c "
    apt-get update && \
    apt-get install -y \
        ros-humble-ros2-control \
        ros-humble-gazebo-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-ur-description \
    && rm -rf /var/lib/apt/lists/*
"
then
    system_message "Universal Robots ROS 2 packages installed successfully."
else
    system_message "Universal Robots ROS 2 packages already installed."
fi

# Clone necessary repositories
system_message "Cloning necessary repositories..."
if docker exec -it "$container_name" bash -c "
    cd /home/dissertation/UR3/ros2_ws/src && \
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git && \
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git && \
    git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
"
then
    system_message "Repositories cloned successfully."
else
    system_message "Repositories already cloned."
fi

# Install dependencies and build the workspace
system_message "Installing dependencies and building the workspace..."
if docker exec -it "$container_name" bash -c "
    cd /home/dissertation/UR3/ros2_ws && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y
"
then
    system_message "Dependencies installed successfully."
else
    system_message "Dependencies already installed."
fi

# Source the ROS 2 setup script and build the workspace
system_message "Sourcing the ROS 2 setup script and building the workspace..."
if docker exec -it "$container_name" bash -c "
    /bin/bash -c 'source /opt/ros/humble/setup.bash && colcon build --event-handlers console_cohesion+ --event-handlers console_direct+'
"
then
    system_message "ROS 2 setup script sourced and workspace built successfully."
else
    system_message "ROS 2 setup script already sourced and workspace already built."
fi

# Source the workspace setup script
system_message "Sourcing the workspace setup script..."
if docker exec -it "$container_name" bash -c "
    echo 'source /home/dissertation/UR3/ros2_ws/install/setup.bash' >> /etc/skel/.bashrc
"
then
    system_message "Workspace setup script sourced successfully."
else
    system_message "Workspace setup script already sourced."
fi

# wait for the user to press enter if -y flag is not set
if [ "$2" != "-y" ]; then
    read -p "Press enter to continue..."
fi

# Set the default user and working directory
system_message "Setting the default user and working directory..."
if docker exec -it "$container_name" bash -c "
    usermod -aG sudo dissertation && \
    mkdir -p /home/dissertation/UR3 && \
    chown -R dissertation:dissertation /home/dissertation/UR3 && \
    echo 'export USER=dissertation' >> /etc/profile && \
    echo 'cd /home/dissertation/UR3' >> /etc/profile
"
then
    system_message "Default user and working directory set successfully."
else
    system_message "Default user and working directory already set."
fi

echo "Initialization script completed successfully."
