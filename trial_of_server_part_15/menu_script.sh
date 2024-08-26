#!/bin/bash

# Function to print menu
print_menu() {
    echo "---------------------------------------"
    echo "Menu:"
    echo "1. Set up COLCON_WS to /workspaces/ur_gazebo and build colcon"
    echo "2. Make a middle_man project and build it"
    echo "3. Make a dummy_node project and build it"
    echo "4. Run dummy_node and middle_man"
    echo "5. Exit"
    echo "---------------------------------------"
}

# Option 1: Set up COLCON_WS and build colcon
setup_colcon_ws() {
    export COLCON_WS="/workspaces/ur_gazebo"
    cd $COLCON_WS
    colcon build
    echo "COLCON_WS set to $COLCON_WS and colcon build completed."
}

# Option 2: Make a middle_man project and build it
create_middle_man() {
    cd $COLCON_WS/src

    # Check if middle_man package exists and remove it
    if [ -d "middle_man" ]; then
        rm -rf middle_man
        echo "Existing middle_man package removed."
    fi

    # Create the middle_man package
    ros2 pkg create middle_man --build-type ament_cmake --dependencies rclcpp trajectory_msgs std_msgs --license Apache-2.0

    # Copy the middle_man_node.cpp file into the src directory
    cp "$(dirname "$0")/middle_man_node.cpp" $COLCON_WS/src/middle_man/src/

    # Build the package
    cd $COLCON_WS
    colcon build --packages-select middle_man
    echo "middle_man project created and built."
}

# Option 3: Make a dummy_node project and build it
create_dummy_node() {
    cd $COLCON_WS/src

    # Check if dummy_node package exists and remove it
    if [ -d "dummy_node" ]; then
        rm -rf dummy_node
        echo "Existing dummy_node package removed."
    fi

    # Create the dummy_node package
    ros2 pkg create dummy_node --build-type ament_cmake --dependencies rclcpp trajectory_msgs std_msgs --license Apache-2.0

    # Copy the dummy_node.cpp file into the src directory
    cp "$(dirname "$0")/dummy_node.cpp" $COLCON_WS/src/dummy_node/src/

    # Build the package
    cd $COLCON_WS
    colcon build --packages-select dummy_node
    echo "dummy_node project created and built."
}

# Option 4: Run the dummy_node and middle_man nodes
run_nodes() {
    source $COLCON_WS/install/setup.bash
    gnome-terminal -- ros2 run middle_man middle_man_node &
    gnome-terminal -- ros2 run dummy_node dummy_node &
    echo "middle_man and dummy_node are running."
}

# Menu loop
while true; do
    print_menu
    read -p "Select an option: " option

    case $option in
        1) setup_colcon_ws ;;
        2) create_middle_man ;;
        3) create_dummy_node ;;
        4) run_nodes ;;
        5) echo "Exiting..." ; exit ;;
        *) echo "Invalid option. Please try again." ;;
    esac
done
