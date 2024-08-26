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
error_check "Failed to install UR Robot Driver."

print_success "UR Robot Driver installed successfully."
# ----------------- Install UR Robot Driver -----------------

print_info "Asking for running the UR Gazebo simulation example..."
read -p "Do you wish to run the UR Gazebo simulation example? [y/n]: " run_example
if [ "$run_example" == "y" ]; then
    read -p $'Choose a simulation example to run [1/2]: \n1. UR Gazebo simulation with control\n2. UR Gazebo simulation with MoveIt\nEnter your choice: ' example_choice
    if [ "$example_choice" == "1" ]; then
        ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
    elif [ "$example_choice" == "2" ]; then
        ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
    else
        print_error "Invalid choice. Exiting..."
        exit 1
    fi
fi

print_ifo "Asking for running the UR Robot Driver example..."
read -p "Do you wish to run the UR Robot Driver example? [y/n]: " run_example
if [ "$run_example" == "y" ]; then
    read "Enter the IP address of the UR robot: " robot_ip
    read "Enter the robot type ( example - ur3e): " robot_type
    ros2 launch ur_robot_driver ur_robot_driver.launch.py robot_ip:="$robot_ip" robot_type:="$robot_type" 
fi