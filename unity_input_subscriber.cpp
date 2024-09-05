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
This program, `unity_input_subscriber.cpp`, is a ROS2 node designed to subscribe to input data from Unity 
and publish joint trajectory commands for a robotic arm. It utilizes ROS2 message types and topics to 
achieve this functionality.

Detailed Explanation:
1. Include Necessary Headers: The program includes headers for ROS2 functionalities and message types 
   required for subscribing and publishing data.
   - Headers:
     ```cpp
     #include "rclcpp/rclcpp.hpp"
     #include "std_msgs/msg/float64_multi_array.hpp"
     #include "trajectory_msgs/msg/joint_trajectory.hpp"
     #include "trajectory_msgs/msg/joint_trajectory_point.hpp"
     ```

2. Define UnityInputSubscriber Class: The `UnityInputSubscriber` class inherits from `rclcpp::Node` and 
   encapsulates the functionality of the ROS2 node.
   - Class Definition:
     ```cpp
     class UnityInputSubscriber : public rclcpp::Node
     ```

3. Constructor: The constructor initializes the node, creates a subscriber to receive input data from Unity, 
   and a publisher to send joint trajectory commands.
   - Constructor:
     ```cpp
     UnityInputSubscriber() : Node("unity_input_subscriber") {
         // Subscriber and Publisher initialization
     }
     ```

4. Callback Function: The callback function processes the received input data and converts it into joint 
   trajectory commands, which are then published.
   - Callback Function:
     ```cpp
     void input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
         // Process input data and publish joint trajectory
     }
     ```

How to Invoke the Program:
To compile and run this ROS2 node, follow these steps:

1. Ensure you have a ROS2 workspace set up. If not, create one:
   ```sh
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
    ```
2. Create a package and copy the code into the respective files:
    ```sh
    ros2 pkg create unity_input_subscriber
    cp [path to unity_input_subscriber.cpp] ~/ros2_ws/src/unity_input_subscriber/src
    cp [path to unity_input_subscriber_CMakeLists.txt] ~/ros2_ws/src/unity_input_subscriber/CMakeLists.txt
    cd ~/ros2_ws
    colcon build
    ```
3. Run the ROS2 node:
    ```sh
    ros2 run unity_input_subscriber unity_input_subscriber
    ```
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class UnityInputSubscriber : public rclcpp::Node
{
public:
    UnityInputSubscriber()
    : Node("unity_input_node")
    {
        // Subscription to receive joint data from Unity
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "unity_input", 10, std::bind(&UnityInputSubscriber::topic_callback, this, std::placeholders::_1));
        
        // Publisher to send the joint data to middle_ware_ur3
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/input_joint_trajectory", 10);

        // Initialize the joint names
        initialize_joint_names();
    }

private:
    // Function to initialize the joint names in the message
    void initialize_joint_names()
    {
        // Example UR robot joint names
        joint_names_ = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };
    }

    // Callback function to handle received data from Unity
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received joint positions from Unity:");

        // Debug: Print all the received values for verification
        for (size_t i = 0; i < msg->data.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Value[%zu]: %f", i, msg->data[i]);
        }

        // Create a JointTrajectory message
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        trajectory_msg.header.stamp = this->now();
        trajectory_msg.joint_names = joint_names_;

        // Create a JointTrajectoryPoint
        trajectory_msgs::msg::JointTrajectoryPoint point;

        // Assuming Unity sends the joint values in the correct order, simply pass them as positions
        for (size_t i = 0; i < msg->data.size(); ++i) {
            point.positions.push_back(msg->data[i]);
        }

        // Set time from start (you can modify this based on your needs)
        point.time_from_start = rclcpp::Duration(1, 0); // 1 second

        // Add the point to the trajectory message
        trajectory_msg.points.push_back(point);

        // Publish the message to middle_ware_ur3
        publisher_->publish(trajectory_msg);

        RCLCPP_INFO(this->get_logger(), "Published joint positions to middle_ware_ur3.");
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

    std::vector<std::string> joint_names_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnityInputSubscriber>());
    rclcpp::shutdown();
    return 0;
}
