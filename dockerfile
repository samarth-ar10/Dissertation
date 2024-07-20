# Use the official Ubuntu 22.04 image as a base
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV SDL_AUDIODRIVER=dummy

# Install dependencies
RUN apt update && apt install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libssl-dev \
    pkg-config \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    wget \
    sudo \
    locales \
    curl \
    gnupg2 \
    software-properties-common 

# ----------------- ROS 2 Installation -----------------
# Set the locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Setup Sources
RUN add-apt-repository universe

# Add the ROS 2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Update package list and install ROS 2
RUN apt-get update && apt-get install -y ros-humble-desktop

# Source ROS 2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/skel/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# ----------------- ROS 2 Installation -----------------

# Clean up
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# ----------------- Gazebo Installation -----------------

# Add the Gazebo GPG key and repository
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update

# Install Gazebo and ros_gz
RUN apt install -y gazebo ros-humble-gazebo-ros-pkgs

# ----------------- Gazebo Installation -----------------
# Clean up
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# ----------------- Custom Installation -----------------
# Make UR3 folder to /home/dissertation for storing all user developed and custom files
RUN mkdir -p /home/dissertation/UR3

# Copy container_menu.sh to /home/dissertation/UR3
COPY container_menu.sh /home/dissertation/UR3
# Making container_menu.sh executable
RUN chmod +x /home/dissertation/UR3/container_menu.sh

# ----------------- Custom Installation -----------------

# Add user named dissertation with sudo privileges
RUN useradd -ms /bin/bash dissertation && \
    usermod -aG sudo dissertation && \
    echo "dissertation ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Create .gazebo directory and set permissions
RUN mkdir -p /home/dissertation/.gazebo && \
    chown -R dissertation:dissertation /home/dissertation/.gazebo

# Set the default user and work directory
USER dissertation
WORKDIR /home/dissertation/UR3

# Set the default shell to bash rather than sh
ENV SHELL /bin/bash

# Set the entrypoint
# ENTRYPOINT ["/home/dissertation/UR3/container_menu.sh"]
