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
This script, `URSIM.sh`, is designed to set up the URSim environment for the UR3 robotic arm. It performs 
system updates, sets the locale, and installs necessary packages for ROS2.

Detailed Explanation:
1. System Update: The script starts by updating the system packages to ensure that the latest versions 
   are installed. This is done using the `apt` package manager.
   - Command:
     ```sh
     sudo apt update && sudo apt upgrade -y
     ```

2. Set ROS Distribution: The script sets the ROS2 distribution to "rolling". This variable is used later 
   in the script for ROS2 installation.
   - Command:
     ```sh
     ROS_DISTRO=rolling
     ```

3. Locale Check and Setup: The script checks the current locale settings and sets the locale to UTF-8 
   to ensure proper internationalization and localization support.
   - Commands:
     ```sh
     locale  # check for UTF-8
     sudo apt update && sudo apt install locales
     sudo locale-gen en_US en_US.UTF-8
     sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
     export LANG=en_US.UTF-8
     locale  # verify settings
     ```

4. Install Necessary Packages: The script installs essential packages required for ROS2 installation. 
   It adds the ROS2 repository and installs ROS2 development tools.
   - Commands:
     ```sh
     sudo apt install -y software-properties-common
     sudo add-apt-repository universe
     sudo apt update && sudo apt install -y curl
     sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
     sudo apt update && sudo apt install -y ros-dev-tools
     ```

How to Invoke the Script:
To invoke the script, you can run it from the command line. For example:
```sh
./URSIM.sh
```
'

#!/bin/bash

# Update the system
sudo apt update && sudo apt upgrade -y
ROS_DISTRO=rolling

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Install necessary packages
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y ros-dev-tools

sudo apt update

sudo apt upgrade -y

sudo apt install -y ros-rolling-desktop ros-rolling-ur-client-library

# Source the ROS 2 setup file
source /opt/ros/rolling/setup.bash

# Install ur package
sudo apt-get install -y ros-${ROS_DISTRO}-ur

# Run the ursim.sh script using ros2 run with a different port
docker run -d --name ursim --network ursim_net -p 30000:29999 -e ROBOT_MODEL=UR3 universalrobots/ursim_e-series