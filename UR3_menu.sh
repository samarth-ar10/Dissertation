#!/bin/bash

current_working_dir=$(pwd)

# Function to print messages
function print_message {
    echo -e "\e[32m$1\e[0m"
}

# Function to log errors
function log_error {
    echo -e "\e[31m$1\e[0m" >&2
    exit 1
}

# Function to install necessary dependencies
function install_dependencies {
    print_message "Installing necessary dependencies..."
    sudo apt update
    sudo apt install -y \
        ros-humble-ros2-controllers \
        ros-humble-ur-description || log_error "Failed to install necessary dependencies."
    print_message "Installing colcon and vcs tools..."
    sudo apt install -y python3-colcon-common-extensions python3-vcstool || log_error "Failed to install colcon and vcs tools."
    print_message "Installing rosdep..."
    sudo apt install -y python3-rosdep || log_error "Failed to install rosdep."
    # remove rosdep source list if it exists
    sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
    sudo rosdep init || log_error "Failed to initialize rosdep."
    rosdep update || log_error "Failed to update rosdep."
}

# Function to get the git library
function get_git_library {
    local cwd=$(pwd)
    print_message "Cloning the Universal Robots ROS2 Driver repository (humble branch)..."
    mkdir -p ./lib/ros2_ws/src
    cd ./lib/ros2_ws || exit
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver || log_error "Failed to clone Universal Robots ROS2 Driver repository."
    # adding urdf folder from main branch to cloned branch and then deleting the main branch
    get_urdf_folder
    cd $current_working_dir/lib/ros2_ws || exit

    vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.humble.repos || log_error "Failed to import repositories."
    
    print_message "Installing dependencies using rosdep..."
    rosdep update
    rosdep install --ignore-src --from-paths src -y || log_error "Failed to install dependencies."
    cd "$cwd"
}

# Function to check for necessary xacro files
function check_xacro_files {
    if [[ ! -f "$current_working_dir/lib/ros2_ws/install/ur_robot_driver/share/ur_robot_driver/urdf/ur.ros2_control.xacro" ]]; then
        log_error "Required xacro file ur.ros2_control.xacro not found. Please ensure the ur_robot_driver package is built correctly."
    fi
}

# Function to get the URDF folder from the main branch
function get_urdf_folder {
    local cwd=$(pwd)
    print_message "Cloning the Universal Robots ROS2 Driver repository (main branch) to get the URDF folder..."
    mkdir -p ./tmp
    cd ./tmp || exit
    git clone -b main https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git || log_error "Failed to clone Universal Robots ROS2 Driver repository (main branch)."
    
    print_message "Copying URDF folder to the main repository..."
    mkdir -p $current_working_dir/urdf
    cp -r Universal_Robots_ROS2_Driver/ur_robot_driver/urdf/* $current_working_dir/urdf || log_error "Failed to copy URDF folder."
    cp -r Universal_Robots_ROS2_Driver/ur_robot_driver/urdf $current_working_dir/lib/ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/ || log_error "Failed to copy URDF folder."

    rm -rf $current_working_dir/lib/ros2_ws/tmp || log_error "Failed to remove temporary folder."
    cd "$cwd"
}

# Function to build the ROS2 workspace
function build_ros2_workspace {
    local cwd=$(pwd)
    cd ./lib/ros2_ws || exit
    print_message "Building the ROS2 workspace..."
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release || log_error "Failed to build the ROS2 workspace."
    source install/setup.bash || log_error "Failed to source the workspace."
    cd "$cwd"
}

# Function to convert xacro to urdf
function convert_xacro_to_urdf {
    local xacro_file=$1
    local urdf_file=$2

    if [[ -z "$xacro_file" || -z "$urdf_file" ]]; then
        log_error "Usage: convert_xacro_to_urdf <input.xacro> <output.urdf>"
    fi

    # print_message "Sourcing ROS2 and workspace setup files..."
    # source /opt/ros/humble/setup.bash || log_error "Failed to source ROS2 setup.bash."
    # source "$current_working_dir/lib/ros2_ws/install/setup.bash" || log_error "Failed to source workspace setup.bash."

    # check_xacro_files

    print_message "Converting $xacro_file to $urdf_file..."
    ros2 run xacro xacro -o "$urdf_file" "$xacro_file" name:=ur3 || log_error "Failed to convert $xacro_file to $urdf_file"
    print_message "Conversion successful: $urdf_file"
}

# Function to clean up created files and folders
function cleanup {
    print_message "Cleaning up..."
    rm -rf ./tmp ./lib ./urdf || log_error "Failed to clean up."
}

# Function to display the menu
function display_menu {
    echo "-----------------------------------------------------------------"
    echo "1. Install Dependencies"
    echo "2. Get Git Library"
    # echo "3. Get URDF Folder"
    echo "3. Build ROS2 Workspace"
    echo "4. Convert Xacro to URDF"
    echo "5. Clean Up"
    echo "6. Exit"
    echo "-----------------------------------------------------------------"
}

# Main script execution
while true; do
    display_menu
    read -p "Choose an option: " choice
    case $choice in
        1)
            install_dependencies
            ;;
        2)
            get_git_library
            ;;
        3)
            build_ros2_workspace
            ;;
        4)
            xacro_file="$current_working_dir/lib/ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/urdf/ur.ros2_control.xacro"
            urdf_file="$current_working_dir/lib/ros2_ws/src/Universal_Robots_ROS2_Driver/ur_robot_driver/urdf/ur3.urdf"
            convert_xacro_to_urdf "$xacro_file" "$urdf_file"
            ;;
        5)
            cleanup
            ;;
        6)
            print_message "Exiting..."
            exit 0
            ;;
        *)
            log_error "Invalid option. Please try again."
            ;;
    esac
done
