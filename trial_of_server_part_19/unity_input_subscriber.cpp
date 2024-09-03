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
