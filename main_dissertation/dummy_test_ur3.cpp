
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
This program, `dummy_test_ur3.cpp`, is a ROS2 node designed to publish dummy joint trajectory commands 
for the UR3 robotic arm. It serves as a test node to simulate joint movements by publishing predefined 
trajectory messages.

Detailed Explanation:
1. Include Necessary Headers: The program includes headers for ROS2 functionalities and message types 
   required for publishing joint trajectory commands.
   - Headers:
     ```cpp
     #include <rclcpp/rclcpp.hpp>
     #include <trajectory_msgs/msg/joint_trajectory.hpp>
     ```

2. Define TestUR3Publisher Class: The `TestUR3Publisher` class inherits from `rclcpp::Node` and encapsulates 
   the functionality of the ROS2 node.
   - Class Definition:
     ```cpp
     class TestUR3Publisher : public rclcpp::Node
     ```

3. Constructor: The constructor initializes the node, creates a publisher to send joint trajectory commands, 
   and sets a timer to periodically publish these commands.
   - Constructor:
     ```cpp
     TestUR3Publisher() : Node("test_ur3_publisher") {
         // Publisher and Timer initialization
     }
     ```

4. Publish Trajectory: The function to publish dummy joint trajectory commands. It creates a trajectory 
   message with predefined joint positions and publishes it.
   - Publish Function:
     ```cpp
     void publish_trajectory() {
         // Create and publish joint trajectory message
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
    ros2 pkg create test_ur3_publisher
    cp [path to dummy_test_ur3.cpp] ~/ros2_ws/src/test_ur3_publisher/dummy_test_ur3.cpp
    cp [path to test_ur3_publisher_CMakeLists.txt] ~/ros2_ws/src/test_ur3_publisher/CMakeLists.txt
    ```
3. Build the package and run the ROS2 node:
    ```sh
    cd ~/ros2_ws
    colcon build
    ros2 run test_ur3_publisher test_ur3_publisher
    ```
*/

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class TestUR3Publisher : public rclcpp::Node
{
public:
    TestUR3Publisher()
        : Node("test_ur3_publisher"), current_point_index_(0)
    {
        // Create a publisher for the /input_joint_trajectory topic
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/input_joint_trajectory", 10);

        // Initialize the trajectory points
        initialize_trajectory_points();

        // Timer to publish messages periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&TestUR3Publisher::publish_trajectory, this));
    }

private:
    void initialize_trajectory_points()
    {
        // Define a series of joint positions

        // First point
        trajectory_msgs::msg::JointTrajectoryPoint point1;
        point1.positions = {0.1, -0.5, 0.5, 0.0, 0.3, -0.2};
        point1.time_from_start = rclcpp::Duration(2, 0);

        // Second point
        trajectory_msgs::msg::JointTrajectoryPoint point2;
        point2.positions = {0.2, -0.6, 0.6, 0.1, 0.4, -0.3};
        point2.time_from_start = rclcpp::Duration(2, 0);

        // Third point
        trajectory_msgs::msg::JointTrajectoryPoint point3;
        point3.positions = {0.3, -0.7, 0.7, 0.2, 0.5, -0.4};
        point3.time_from_start = rclcpp::Duration(2, 0);

        // Add points to the vector
        trajectory_points_.push_back(point1);
        trajectory_points_.push_back(point2);
        trajectory_points_.push_back(point3);
    }

    void publish_trajectory()
    {
        // Create a JointTrajectory message
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.header.stamp = this->now();
        message.joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"};

        // Add the current trajectory point to the message
        message.points.push_back(trajectory_points_[current_point_index_]);

        // Log and publish the message
        RCLCPP_INFO(this->get_logger(), "Publishing trajectory point %zu", current_point_index_);
        publisher_->publish(message);

        // Move to the next point in the sequence
        current_point_index_ = (current_point_index_ + 1) % trajectory_points_.size();
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points_;
    size_t current_point_index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestUR3Publisher>());
    rclcpp::shutdown();
    return 0;
}
