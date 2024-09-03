#!/bin/bash

# Function to print menu
print_menu() {
    echo "---------------------------------------"
    echo "Menu:"
    echo "1. Set up COLCON_WS to ~/workspaces/ur_gazebo and build colcon"
    echo "2. Make a middle_ware_ur3 project and build it"
    echo "3. Make a dummy_test_ur3 project and build it"
    echo "4. Run dummy_test_ur3 and middle_ware_ur3"
    echo "5. Exit"
    echo "---------------------------------------"
}

# Option 1: Set up COLCON_WS and build colcon
setup_colcon_ws() {
    export COLCON_WS=~/workspaces/ur_gazebo
    mkdir -p $COLCON_WS/src
    cd $COLCON_WS
    colcon build
    echo "COLCON_WS set to $COLCON_WS and colcon build completed."
}

# Option 2: Make a middle_ware_ur3 project and build it
create_middle_ware_ur3() {
    cd $COLCON_WS/src
    # Check if middle_ware_ur3 package exists and remove it
    if [ -d "middle_ware_ur3" ]; then
        rm -rf middle_ware_ur3
        echo "Existing middle_ware_ur3 package removed."
    fi
    # Create the middle_ware_ur3 package
    ros2 pkg create middle_ware_ur3 --build-type ament_cmake --dependencies rclcpp trajectory_msgs std_msgs --license Apache-2.0
    # Ensure the source directory exists
    if [ ! -d "$COLCON_WS/src/middle_ware_ur3/src" ]; then
        mkdir -p $COLCON_WS/src/middle_ware_ur3/src
    fi
    
    # Copy the middle_ware_ur3.cpp file into the src directory
    cp "$(dirname "$0")/middle_ware_ur3.cpp" $COLCON_WS/src/middle_ware_ur3/src/
    if [ -f "$COLCON_WS/src/middle_ware_ur3/src/middle_ware_ur3.cpp" ]; then
        echo "middle_ware_ur3.cpp copied successfully."
    else
        echo "Error: middle_ware_ur3.cpp not found. Copy failed."
    fi
    # Generate the CMakeLists.txt file
    cat <<EOL > $COLCON_WS/src/middle_ware_ur3/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(middle_ware_ur3)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(middle_ware_ur3_node src/middle_ware_ur3.cpp)

ament_target_dependencies(middle_ware_ur3_node rclcpp trajectory_msgs std_msgs)

install(TARGETS
  middle_ware_ur3_node
  DESTINATION lib/\${PROJECT_NAME})

ament_package()
EOL
    # Build the package
    cd $COLCON_WS
    colcon build --packages-select middle_ware_ur3
    echo "middle_ware_ur3 project created and built."
}


# Option 3: Make a dummy_test_ur3 project and build it
create_dummy_test_ur3() {
    cd $COLCON_WS/src
    # Check if dummy_test_ur3 package exists and remove it
    if [ -d "dummy_test_ur3" ]; then
        rm -rf dummy_test_ur3
        echo "Existing dummy_test_ur3 package removed."
    fi
    # Create the dummy_test_ur3 package
    ros2 pkg create dummy_test_ur3 --build-type ament_cmake --dependencies rclcpp trajectory_msgs std_msgs --license Apache-2.0
    # Copy the test_ur3_publisher.cpp file into the src directory
    mkdir -p $COLCON_WS/src/dummy_test_ur3/src
    cp "$(dirname "$0")/test_ur3_publisher.cpp" $COLCON_WS/src/dummy_test_ur3/src/
    # Generate the CMakeLists.txt file
    cat <<EOL > $COLCON_WS/src/dummy_test_ur3/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(dummy_test_ur3)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(test_ur3_publisher src/test_ur3_publisher.cpp)

ament_target_dependencies(test_ur3_publisher rclcpp trajectory_msgs)

install(TARGETS
  test_ur3_publisher
  DESTINATION lib/\${PROJECT_NAME})

ament_package()
EOL
    # Build the package
    cd $COLCON_WS
    colcon build --packages-select dummy_test_ur3
    echo "dummy_test_ur3 project created and built."
}

# Option 4: Run the dummy_test_ur3 and middle_ware_ur3 nodes
run_nodes() {
    if [ -f "$COLCON_WS/install/setup.bash" ]; then
        source $COLCON_WS/install/setup.bash
        
        # Run middle_ware_ur3 node in the background and get its PID
        nohup ros2 run middle_ware_ur3 middle_ware_ur3_node > middle_ware_ur3_node.log 2>&1 &
        MIDDLE_WARE_UR3_PID=$!
        echo "middle_ware_ur3_node is running with PID $MIDDLE_WARE_UR3_PID."
        
        # Run dummy_test_ur3 node in the background and get its PID
        nohup ros2 run dummy_test_ur3 test_ur3_publisher > test_ur3_publisher.log 2>&1 &
        DUMMY_TEST_UR3_PID=$!
        echo "test_ur3_publisher is running with PID $DUMMY_TEST_UR3_PID."
        
        echo "middle_ware_ur3 and dummy_test_ur3 nodes are running."
    else
        echo "Error: $COLCON_WS/install/setup.bash not found. Please build the workspace first."
    fi
}

# Menu loop
while true; do
    print_menu
    read -p "Select an option: " option

    case $option in
        1) setup_colcon_ws ;;
        2) create_middle_ware_ur3 ;;
        3) create_dummy_test_ur3 ;;
        4) run_nodes ;;
        5) echo "Exiting..." ; exit ;;
        *) echo "Invalid option. Please try again." ;;
    esac
done
