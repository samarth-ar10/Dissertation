# 
# MIT License
# 
# Author: Samarth
# Contact: psxs2@nottingham.ac.uk
# 
# This code is prepared for the dissertation project at the University of Nottingham.
# The University of Nottingham has governance over this code.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

FROM ubuntu:22.04

# setup environment
ENV LANG C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO humble
ENV DISPLAY :0

# Install necessary packages
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    wget \
    git \
    python3-pip \
    python3-apt \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    colcon-common-extensions \
    rosdep \
    vcstool \
    rosinstall_generator

# Rosdep init
RUN rosdep init
RUN rosdep update

# set system timezone to LONDON - This is based on user preference
RUN ln -fs /usr/share/zoneinfo/Europe/London /etc/localtime

COPY ur3_setup.sh /root/ur3_setup.sh
RUN chmod +x /root/ur3_setup.sh

COPY dummy_test_ur3.cpp /root/dummy_test_ur3.cpp
COPY dummy_test_ur3_CMakeLists.txt /root/dummy_test_ur3_CMakeLists.txt
COPY middle_ware_ur3.cpp /root/middle_ware_ur3.cpp
COPY middle_ware_ur3_CMakeLists.txt /root/middle_ware_ur3_CMakeLists.txt
COPY unity_input_subscriber.cpp /root/unity_input_subscriber.cpp
COPY unity_input_subscriber_CMakeLists.txt /root/unity_input_subscriber_CMakeLists.txt

# Set the working directory
WORKDIR /root

# Source ROS 2 and workspace setup files
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Expose port for ROS
EXPOSE 11311
# Expose port for Gazebo
EXPOSE 11345
# Expose port for ROS2-TCP-Endpoint
EXPOSE 10000

# # Enter the container with and run ur3_setup.sh
# CMD ["/bin/bash", "-c", "/root/ur3_setup.sh"]

# Enter the container in /bin/bash, print the container ip address so that the host can connect with it and then run ur3_setup.sh 
# CMD ["/bin/bash", "-c", "echo 'Container IP Address: ' && hostname -I && /root/ur3_setup.sh sim"]
# CMD ["/bin/bash", "-c", "echo 'Container IP Address: ' && hostname -I && /root/ur3_setup.sh driver ur3e 172.22.2.1"]
# Run /root/ur3_setup.sh with the argument sim if you want to run the simulation
# Run /root/ur3_setup.sh with the argument driver if you want to run the driver followed by the ur_type and the ip address of the robot

# # open terminal at /root
CMD ["/bin/bash"]