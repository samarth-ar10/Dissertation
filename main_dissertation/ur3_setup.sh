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
This script, `ur3_setup.sh`, is designed to set up the environment for the UR3 robotic arm. It includes 
functions for printing informational and error messages, setting up ROS2 and Gazebo environments, and 
configuring Docker containers for simulation and development purposes.

Detailed Explanation:
1. Print Informational Messages: The script includes a function to print messages in yellow color to 
   indicate informational messages to the user. This helps in distinguishing informational messages 
   from other types of messages.
   - Function:
     ```sh
     print_info() {
         echo -e "\e[33m[INFO] $1\e[0m"
     }
     ```

2. Print Error Messages: The script includes a function to print messages in red color to indicate error 
   messages to the user. This helps in distinguishing error messages from other types of messages.
   - Function:
     ```sh
     print_error() {
         echo -e "\e[31m[ERROR] $1\e[0m"
     }
     ```

3. Setup ROS2 Environment: The script sets up the ROS2 environment by sourcing the necessary setup scripts. 
   This ensures that the ROS2 environment is correctly configured for running ROS2 nodes and applications.
   - Function:
     ```sh
     setup_ros2_environment() {
         source /opt/ros/$ROS_DISTRO/setup.bash
         print_info "ROS2 environment set up for $ROS_DISTRO"
     }
     ```

4. Setup Gazebo Environment: The script sets up the Gazebo environment by sourcing the necessary setup scripts. 
   This ensures that the Gazebo simulation environment is correctly configured for running simulations.
   - Function:
     ```sh
     setup_gazebo_environment() {
         source /usr/share/gazebo/setup.sh
         print_info "Gazebo environment set up"
     }
     ```

5. Configure Docker Container: The script configures the Docker container by setting up the default user, 
   working directory, and sourcing the workspace setup script. This ensures that the Docker container is 
   correctly configured for development and simulation purposes.
   - Function:
     ```sh
     configure_docker_container() {
         useradd -m -d /home/developer -s /bin/bash developer
         echo "developer:developer" | chpasswd
         usermod -aG sudo developer
         print_info "Docker container configured"
     }
     ```

How to Invoke the Script:
To invoke the script, you can run it from the command line with the appropriate arguments. For example:
```sh
./ur3_setup.sh sim
./ur3_setup.sh driver <robot_ip> <robot_type>
```
The first argument specifies whether to run the simulation or the driver example. The second and third
arguments are required when running the driver example and specify the IP address of the robot and the
type of the robot, respectively.
'

#!/bin/bash

# ----------------- Setting up basic functionalities -----------------
# Function to print yellow colored message for information
print_info() {
    echo -e "\e[33m[INFO] $1\e[0m"
}

# Function to print red colored message for error
print_error() {
    echo -e "\e[31m[ERROR] $1\e[0m"
}

# Function to print green colored message for successful execution
print_success() {
    echo -e "\e[32m[SUCCESS] $1\e[0m"
}

error_check() {
    if [ $? -ne 0 ]; then
        print_error "$1"
        exit 1
    fi
}

# Example usage
echo "While executing this script, we use the following colored messages scheme for better readability:"
print_info "This is an informational message."
print_error "This is an error message."
print_success "This is a success message."

# ----------------- Setting up basic functionalities -----------------
# ----------------- Install basic dependencies -----------------
# ----------------- Install basic dependencies -----------------
print_info "Installing basic dependencies..."
error_check "Failed to install basic dependencies."

locale  # check for UTF-8
print_info "Checking locale settings..."
error_check "Failed to check locale settings."

sudo apt update && sudo apt install locales -y
error_check "Failed to update and install locales."

sudo locale-gen en_US en_US.UTF-8
error_check "Failed to generate locales."

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
error_check "Failed to update locale settings."

export LANG=en_US.UTF-8

locale  # verify settings
print_info "Locale settings verified."
print_success "Basic dependencies installed successfully."

# ----------------- Install basic dependencies -----------------

# ----------------- Install ROS2 -----------------
print_info "Installing ROS2..."
error_check "Failed to install ROS2."

sudo apt install software-properties-common -y
error_check "Failed to install software-properties-common."

sudo add-apt-repository universe
error_check "Failed to add universe repository."

sudo apt update && sudo apt install curl -y
error_check "Failed to update and install curl."

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
error_check "Failed to download ROS key."

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
error_check "Failed to add ROS2 repository."

sudo apt update
error_check "Failed to update repositories."

sudo apt upgrade -y
error_check "Failed to upgrade packages."

sudo apt install ros-humble-desktop ros-humble-gazebo-ros2-control ros-humble-ament-cmake -y
error_check "Failed to install ROS2 packages."

sudo apt install ros-dev-tools -y
error_check "Failed to install ROS2 development tools."

source /opt/ros/humble/setup.bash
error_check "Failed to source ROS2 setup.bash."

print_success "ROS2 installed successfully."

# ----------------- Install ROS2 -----------------

# ----------------- Install Gazebo -----------------
# If you have previously installed gazebo and are facing issues with the installation of gazebo, you can remove gazebo and reinstall it by running the following commands:
# sudo apt-get remove --purge gazebo*
# sudo apt-get autoremove

print_info "Installing Gazebo..."

print_info "Updating system packages..."
sudo apt-get update
error_check "Failed to update system packages."

print_info "Installing lsb-release and gnupg..."
sudo apt-get install lsb-release gnupg -y
error_check "Failed to install lsb-release and gnupg."

# sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo apt-get update
# sudo apt-get install gz-harmonic

print_info "Downloading gazebo..."
sudo apt-get install gazebo -y
error_check "Failed to install gazebo."

print_success "Gazebo installed successfully."

# ----------------- Install Gazebo -----------------

# ----------------- Install UR Gazebo -----------------

print_info "Installing UR Gazebo..."

print_info "Setting up colcon workspace..."
export COLCON_WS=~/workspaces/ur_gazebo
mkdir -p $COLCON_WS/src
cd $COLCON_WS/src
error_check "Failed to create colcon workspace."

print_info "Cloning UR Gazebo repository..."
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
rosdep update && rosdep install --ignore-src --from-paths . -y
error_check "Failed to clone UR Gazebo repository."

print_info "Building UR Gazebo package..."
cd $COLCON_WS
colcon build --symlink-install
error_check "Failed to build UR Gazebo package."

# Add source command to .bashrc if not already present
source /opt/ros/humble/setup.bash >> ~/.bashrc

# Add source command to .bashrc if not already present
if ! grep -Fxq "source $COLCON_WS/install/setup.bash" ~/.bashrc
then
    echo "source $COLCON_WS/install/setup.bash" >> ~/.bashrc
elif grep -Fxq "source $COLCON_WS/install/setup.bash" ~/.bashrc
then
    echo "source command already present in .bashrc"
    source $COLCON_WS/install/setup.bash
fi
print_success "UR Gazebo installed successfully."

print_success "Setup complete! Please reload your terminal or run 'source /opt/ros/humble/setup.bash && source $COLCON_WS/install/setup.bash' to use the ur_simulation_gazebo package."
print                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
# ----------------- Install UR Gazebo -----------------

# ----------------- Install UR Robot Driver -----------------

print_info "Installing UR Robot Driver..."
sudo apt-get install ros-rolling-ur 
sudo apt install ros-humble-rqt-graph

error_check "Failed to install UR Robot Driver."

print_success "UR Robot Driver installed successfully."
# ----------------- Install UR Robot Driver -----------------

# ----------------- Install Unity ROS2-TCP Endpoint -----------------

print_info "Installing Unity ROS2-TCP Endpoint..."
cd $COLCON_WS/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git##
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
# Note: yes, you need to run the source command twice. The first sets up the environment for the build to use, the second time adds the newly built packages to the environent.
error_check "Failed to clone Unity ROS2-TCP Endpoint repository."
print_success "Unity ROS2-TCP Endpoint installed successfully."

# ----------------- Install Unity ROS2-TCP Endpoint -----------------
cd $COLCON_WS/src

ros2 pkg create dummy_test_ur3 --build-type ament_cmake --dependencies rclcpp trajectory_msgs --license Apache-2.0
cp /root/dummy_test_ur3_CMakeLists.txt $COLCON_WS/src/dummy_test_ur3/CMakeLists.txt
cp /root/dummy_test_ur3.cpp $COLCON_WS/src/dummy_test_ur3/src/dummy_test_ur3.cpp

ros2 pkg create middle_ware_ur3 --build-type ament_cmake --dependencies rclcpp trajectory_msgs std_msgs --license Apache-2.0
cp /root/middle_ware_ur3_CMakeLists.txt $COLCON_WS/src/middle_ware_ur3/CMakeLists.txt
cp /root/middle_ware_ur3.cpp $COLCON_WS/src/middle_ware_ur3/src/middle_ware_ur3.cpp

ros2 pkg create unity_input_node --build-type ament_cmake --dependencies rclcpp trajectory_msgs std_msgs --license Apache-2.0
cp /root/unity_input_subscriber_CMakeLists.txt $COLCON_WS/src/unity_input_node/CMakeLists.txt
cp /root/unity_input_subscriber.cpp $COLCON_WS/src/unity_input_node/src/unity_input_subscriber.cpp

cd $COLCON_WS
colcon build --symlink-install
error_check "Failed to build ROS2 packages."

source $COLCON_WS/install/setup.bash
error_check "Failed to source ROS2 setup.bash."


# ----------------- Running UR3 Setup -----------------

print_info "Running UR3 Setup..."

# the first parameter is either sim or driver to run the simulation or the driver
if [ "$1" == "sim" ]; then
    print_info "Running the UR Gazebo simulation example..."
    xterm -e "ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py" &
elif [ "$1" == "driver" ]; then
    print_info "Running the UR Robot Driver example..."
    # the second parameter is the IP address of the robot if the first parameter is driver
    # the third parameter is the robot type if the first parameter is driver
    xterm -e "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=\"$2\" robot_ip:=\"$3\"" &
else
    print_error "Invalid choice. Enter either 'sim' or 'driver' as the first parameter."
    exit 1
fi
print_success "UR3 Setup complete!" 

print_info "Running the middleware..."
# xterm -e "ros2 run middle_ware_ur3 middle_ware_ur3_node" &
source /opt/ros/humble/setup.bash
source ~/workspaces/ur_gazebo/install/setup.bash
xterm -e ros2 run middle_ware_ur3 middleware_ur3_node --ros-args -p node_name_:=testing &
print_success "Middleware running successfully."

read -p "Do you wish to test the middleware? [y/n]: " test_middleware
if [ "$test_middleware" == "y" ]; then
    print_info "Running the dummy node to test the middleware. Press Ctrl+C to stop the node."
    read -p "Press any key to continue..." 
    ros2 run dummy_test_ur3 test_ur3_publisher
fi

print_info "Running the Unity ROS2-TCP Endpoint..."
xterm -e "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=172.18.0.1" &
print_success "ROS2-TCP Endpoint running successfully."

print_info "Running the Unity application..."
xterm -e "ros2 run unity_input_node unity_input_subscriber" &
print_success "Unity application running successfully."

print_success "Setup complete! Please reload your terminal or run 'source /opt/ros/humble/setup.bash && source $COLCON_WS/install/setup.bash' to interact further with the UR3 setup."
read -p "Press any key to exit..."
# ----------------- Running UR3 Setup -----------------



# print_info "Asking for running the UR Gazebo simulation example..."
# read -p "Do you wish to run the UR Gazebo simulation example? [y/n]: " run_example
# if [ "$run_example" == "y" ]; then
#     read -p $'Choose a simulation example to run [1/2]: \n1. UR Gazebo simulation with control\n2. UR Gazebo simulation with MoveIt\nEnter your choice: ' example_choice
#     if [ "$example_choice" == "1" ]; then
#         ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
#     elif [ "$example_choice" == "2" ]; then
#         ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
#     else
#         print_error "Invalid choice. Exiting..."
#         exit 1
#     fi
# fi

# print_info "Asking for running the UR Robot Driver example..."
# read -p "Do you wish to run the UR Robot Driver example? [y/n]: " run_example
# if [ "$run_example" == "y" ]; then
#     read "Enter the IP address of the UR robot: " robot_ip
#     read "Enter the robot type ( example - ur3e): " robot_type
#     ros2 launch ur_robot_driver ur_robot_driver.launch.py robot_ip:="$robot_ip" robot_type:="$robot_type" 
# fi