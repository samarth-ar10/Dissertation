# Use Ubuntu Jammy as the base image
FROM ubuntu:jammy

# Set the locale to support UTF-8[^1^][1]
ENV LANG en_US.UTF-8[^2^][2]
ENV LC_ALL en_US.UTF-8
# set it as non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# ---------------- universal dependies ----------------
# Enable the Ubuntu Universe repository
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe \
    && apt-get update

RUN apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    sudo \
    wget \
    git \
    nano \
    build-essential \
    cmake \
    lsb-release \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*
# python3-pip \
# python3-rosdep \
# python3-vcstool

# ---------------- ROS 2 ----------------
# source: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# Add the ROS 2 GPG key and repository
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys F42ED6FBAB17C654 && \
    sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list'

# Update the apt repository caches[^3^][3]
RUN apt-get update

# Install ROS 2 Humble Hawksbill Desktop
RUN apt-get install -y ros-humble-desktop

# Source the ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Expose the necessary ports
EXPOSE 11311

# Source the ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# ---------------- ROS 2 ----------------

# ---------------- Gazebo ----------------
# source: https://gazebosim.org/docs/harmonic/install_ubuntu/

# # Add the Gazebo repository to sources list
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# # Install Gazebo Harmonic metapackage
# RUN apt-get update && apt-get install -y gz-harmonic

# Add the Gazebo repository GPG key
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add the Gazebo repository to sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic metapackage
RUN apt-get update && apt-get install -y gz-harmonic

# Attaching display to the container
ENV DISPLAY=:0


# # ---------------- Gazebo ----------------


# set workdir
WORKDIR /root
