# UR3 Robotic Arm Project

## Overview

This project is designed to set up and configure the UR3 robotic arm environment. It includes various functionalities such as running ROS2 nodes, setting up Gazebo simulations, and configuring Docker containers for simulation and development purposes.

## Project Structure



### Key Components

- **docker_menu.sh**: Script to manage Docker containers for the project.
- **dockerfile**: Dockerfile to build the Docker image for the UR3 environment.
- **dummy_test_ur3_CMakeLists.txt**: CMake configuration for the dummy test node.
- **dummy_test_ur3.cpp**: ROS2 node for testing the UR3 middleware.
- **host_gazebo.sh**: Script to launch Gazebo simulation on the host machine.
- **kernel_rt.sh**: Script for setting up the real-time kernel. *Note that in case of running Ubuntu, the script might not be needed as ubuntu provides real-time kernel and prevents the need for setting up the real-time kernel.*
- **middle_ware_ur3_CMakeLists.txt**: CMake configuration for the middleware node.
- **middle_ware_ur3.cpp**: Middleware node for the UR3 robotic arm.
- **unity_input_subscriber_CMakeLists.txt**: CMake configuration for the Unity input subscriber node.
- **unity_input_subscriber.cpp**: ROS2 node to subscribe to Unity inputs.
- **ur3_local_setup.sh**: Script for local setup of the UR3 environment.
- **ur3_setup.sh**: Main setup script for the UR3 environment.

## Architecture

### ROS2 Nodes

- **TestUR3Publisher**: Defined in [`dummy_test_ur3.cpp`](command:_github.copilot.openRelativePath?%5B%7B%22scheme%22%3A%22file%22%2C%22authority%22%3A%22%22%2C%22path%22%3A%22%2Fhome%2Fsamarth%2FGitHub%2FDissertation%2Fmain_dissertation%2Fdummy_test_ur3.cpp%22%2C%22query%22%3A%22%22%2C%22fragment%22%3A%22%22%7D%5D "/home/samarth/GitHub/Dissertation/main_dissertation/dummy_test_ur3.cpp"), this node is used to test the middleware by publishing test messages.
- **MiddlewareUR3**: Defined in [`middle_ware_ur3.cpp`](command:_github.copilot.openRelativePath?%5B%7B%22scheme%22%3A%22file%22%2C%22authority%22%3A%22%22%2C%22path%22%3A%22%2Fhome%2Fsamarth%2FGitHub%2FDissertation%2Fmain_dissertation%2Fmiddle_ware_ur3.cpp%22%2C%22query%22%3A%22%22%2C%22fragment%22%3A%22%22%7D%5D "/home/samarth/GitHub/Dissertation/main_dissertation/middle_ware_ur3.cpp"), this node handles the middleware functionalities for the UR3 robotic arm. It acts as a bridge between the UR3 hardware/simulation and rest of the ROS2 nodes. With this method you hide the complexity of the UR3 hardware/simulation from the rest of the ROS2 nodes and regardless of the hardware/simulation you can use the same ROS2 nodes. 
- **UnityInputSubscriber**: Defined in [`unity_input_subscriber.cpp`](command:_github.copilot.openRelativePath?%5B%7B%22scheme%22%3A%22file%22%2C%22authority%22%3A%22%22%2C%22path%22%3A%22%2Fhome%2Fsamarth%2FGitHub%2FDissertation%2Fmain_dissertation%2Funity_input_subscriber.cpp%22%2C%22query%22%3A%22%22%2C%22fragment%22%3A%22%22%7D%5D "/home/samarth/GitHub/Dissertation/main_dissertation/unity_input_subscriber.cpp"), this node subscribes to inputs from Unity using ROS-TCP endpoint and sends the inputs to the MiddlewareUR3 node.

### Docker

- **dockerfile**: Used to build a Docker image that contains all the necessary dependencies and configurations for running the UR3 environment.
- **docker_menu.sh**: Provides a CLI menu interface to manage Docker containers, including starting and stopping containers, and running specific scripts inside the containers.

### Gazebo

- **host_gazebo.sh**: Script to connect to the Gazebo simulation running in a container from the host machine.

## How to Work with the Project

### Prerequisites

- Docker
- ROS2 Humble
- Gazebo
- Unity

### Setup

1. **Clone the Repository**

   ```sh
   git clone <repository_url>
   cd <repository_directory>
    ```
2. **Run the Setup Script on the host machine without Docker**

   ```sh
   ./ur3_local_setup.sh
   ```
3. **Run the Setup Script on the host machine with Docker**

   ```sh
   ./docker_menu.sh
   ```
   within that menu, select the option to build the Docker image and then run the Docker container.
   In the docker container run the following command to setup the UR3 environment:
   ```sh
    ./ur3_setup.sh sim # for simulation
    ./ur3_setup.sh driver <ur_type> <robot_ip> # for driver
    ``` 
    NOTE: Replace `<ur_type>` with the type of UR robot (e.g., UR3, UR5, UR10) and `<robot_ip>` with the IP address of the robot.

   
4. **Prepare Unity for ROS-TCP Communication**

   - Install the ROS-TCP Connector in Unity.
   - Set up the ROS-TCP Connector to connect to the ROS2 nodes running in the Docker container.
   - NOTE: There is a bug in the ROS-TCP Connector for VR that prevents it from connecting from connecting from any other IP address other than the first one entered from the option in the menu. To fix this, you can add a component to any game object of the ROS-TCP Connector and enter the new IP address.
    
