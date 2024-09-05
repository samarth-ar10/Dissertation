/*
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
*/

/*
This program, `middle_ware_ur3.cpp`, is a ROS2 node designed to act as middleware for the UR3 robotic arm. 
It subscribes to joint trajectory commands and publishes status messages. The program ensures thread-safe 
operations using mutexes and manages a set of active joints.

Detailed Explanation:
1. Include Necessary Headers: The program includes headers for ROS2 functionalities, message types, 
   and standard libraries required for thread safety and set operations.
   - Headers:
     ```cpp
     #include <rclcpp/rclcpp.hpp>
     #include <trajectory_msgs/msg/joint_trajectory.hpp>
     #include <std_msgs/msg/string.hpp>
     #include <set>
     #include <mutex>
     ```

2. Define MiddlewareUR3 Class: The `MiddlewareUR3` class inherits from `rclcpp::Node` and encapsulates 
   the functionality of the ROS2 node.
   - Class Definition:
     ```cpp
     class MiddlewareUR3 : public rclcpp::Node
     ```

3. Constructor: The constructor initializes the node, creates a subscriber to receive joint trajectory 
   commands, and a publisher to send status messages. It also initializes the set of active joints and 
   the mutex for thread safety.
   - Constructor:
     ```cpp
     MiddlewareUR3() : Node("middleware_ur3") {
         // Subscriber and Publisher initialization
         // Set and Mutex initialization
     }
     ```

4. Callback Function: The callback function processes the received joint trajectory commands and updates 
   the set of active joints. It ensures thread-safe operations using the mutex.
   - Callback Function:
     ```cpp
     void trajectory_callback(const trajectory_msgs::msg::Joint_trajectory::SharedPtr msg) {
         // Process joint trajectory and update active joints
     }
     ```

5. Publish Status: The function to publish status messages about the active joints. It ensures thread-safe 
   access to the set of active joints using the mutex.
   - Publish Function:
     ```cpp
     void publish_status() {
         // Publish status messages
     }
     ```

How to Invoke the Program:
To compile and run this ROS2 node, follow these steps:

1. Ensure you have a ROS2 workspace set up. If not, create one:
   ```sh
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
    ```
2. Create a package and copy the code into the respective files:
    ```sh
    ros2 pkg create middle_ware_ur3
    cp [path to middle_ware_ur3.cpp] ~/ros2_ws/src/middle_ware_ur3/middle_ware_ur3.cpp
    cp [path to middle_ware_ur3_CMakeLists.txt] ~/ros2_ws/src/middle_ware_ur3/CMakeLists.txt
    cd ~/ros2_ws
    colcon build
    ```
3. Run the ROS2 node:
    ```sh
    ros2 run middle_ware_ur3 middle_ware_ur3
    ```
*/

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <set>
#include <mutex>

class MiddlewareUR3 : public rclcpp::Node
{
public:
    MiddlewareUR3()
        : Node("middle_ware_ur3"), node_name_(""), control_node_("")
    {
        // Setup name registration and conflict resolution
        setup_node_name();

        // Publisher to the simulation's joint_trajectory_controller topic
        sim_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // Publisher to the real robot's joint_trajectory_controller topic
        real_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);

        // Publisher to the virtual trajectory topic
        virtual_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/virtual_joint_trajectory", 10);

        // Publisher and subscription for node identification and privilege management
        name_pub_ = this->create_publisher<std_msgs::msg::String>("/node_name_broadcast", 10);
        name_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/node_name_broadcast", 10, std::bind(&MiddlewareUR3::name_callback, this, std::placeholders::_1));

        // Subscription to the topic where commands are published
        subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/input_joint_trajectory", 10,
            std::bind(&MiddlewareUR3::trajectory_callback, this, std::placeholders::_1));

        // Timer to broadcast the node's name periodically
        name_broadcast_timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&MiddlewareUR3::broadcast_name, this));

        RCLCPP_INFO(this->get_logger(), "MiddlewareUR3 has been started with name: %s", node_name_.c_str());
    }

private:
    std::string node_name_;
    std::set<std::string> known_nodes_;
    std::string control_node_;
    std::mutex name_mutex_;

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr sim_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr real_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr virtual_publisher_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr name_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr name_sub_;
    rclcpp::TimerBase::SharedPtr name_broadcast_timer_;

    void setup_node_name()
    {
        while (true)
        {
            std::cout << "Enter a unique name for this node: ";
            std::getline(std::cin, node_name_);

            std::lock_guard<std::mutex> lock(name_mutex_);
            if (known_nodes_.find(node_name_) != known_nodes_.end())
            {
                RCLCPP_WARN(this->get_logger(), "Name '%s' already exists on the network. Please choose a different name.", node_name_.c_str());
            }
            else
            {
                break;
            }
        }
    }

    void name_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(name_mutex_);
        std::string received_name = msg->data;
        if (received_name != node_name_ && known_nodes_.find(received_name) == known_nodes_.end())
        {
            known_nodes_.insert(received_name);
            RCLCPP_INFO(this->get_logger(), "Discovered new node: %s", received_name.c_str());
        }
    }

    void broadcast_name()
    {
        std_msgs::msg::String msg;
        msg.data = node_name_;
        name_pub_->publish(msg);
    }

    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // Ensure that this node has control, or just allow listening
        {
            std::lock_guard<std::mutex> lock(name_mutex_);
            if (!control_node_.empty() && control_node_ != node_name_)
            {
                RCLCPP_WARN(this->get_logger(), "This node does not have control privileges. Ignoring trajectory command.");
                return;
            }
            if (control_node_.empty())
            {
                control_node_ = node_name_;
            }
        }

        // Log the received message
        RCLCPP_INFO(this->get_logger(), "Received trajectory message with %zu joints.", msg->joint_names.size());
        for (size_t i = 0; i < msg->joint_names.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Joint %s: Position %f", msg->joint_names[i].c_str(), msg->points[0].positions[i]);
        }

        // Create separate messages for the simulation and real robot
        auto sim_msg = *msg;                    // Copy the incoming message for simulation
        sim_msg.header.stamp = rclcpp::Time(0); // Set stamp to 0 for simulation

        auto real_msg = *msg;                // Copy the incoming message for the real robot
        real_msg.header.stamp = this->now(); // Use current time for the real robot

        // Combine both for the virtual trajectory
        auto virtual_msg = *msg;
        virtual_msg.header.stamp = this->now(); // Set current time for virtual trajectory

        bool sim_published = false;
        bool real_published = false;

        // Check if the simulation publisher is connected
        if (sim_publisher_->get_subscription_count() > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing to simulation.");
            sim_publisher_->publish(sim_msg);
            sim_published = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Simulation topic not available.");
        }

        // Check if the real robot publisher is connected
        if (real_publisher_->get_subscription_count() > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing to real robot.");
            real_publisher_->publish(real_msg);
            real_published = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Real robot topic not available.");
        }

        if (sim_published || real_published)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing virtual trajectory.");
            virtual_publisher_->publish(virtual_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to publish trajectory message to any target.");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiddlewareUR3>());
    rclcpp::shutdown();
    return 0;
}
