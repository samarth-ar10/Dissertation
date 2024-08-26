// middle_man_node.cpp is a ROS 2 node that subscribes to the /input_joint_trajectory topic and republishes the received message to the /joint_trajectory_controller/joint_trajectory topic.
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class MiddleManNode : public rclcpp::Node
{
public:
    MiddleManNode()
    : Node("middle_man_node")
    {
        // Subscription to the topic where commands are published
        subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/input_joint_trajectory", 10,
            std::bind(&MiddleManNode::trajectory_callback, this, std::placeholders::_1));

        // Publisher to the joint_trajectory_controller topic
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        RCLCPP_INFO(this->get_logger(), "MiddleManNode has been started.");
    }

private:
    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // Log the received message
        RCLCPP_INFO(this->get_logger(), "Received trajectory message with %zu joints.", msg->joint_names.size());
        for (size_t i = 0; i < msg->joint_names.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Joint %s: Position %f", msg->joint_names[i].c_str(), msg->points[0].positions[i]);
        }

        // Create a new message to publish
        auto new_msg = trajectory_msgs::msg::JointTrajectory();
        new_msg.header.stamp.sec = 0;  // Set sec to 0 because otherwise the simulation will not run (based on testing)
        new_msg.header.stamp.nanosec = 0;  // Set nanosec to 0 because otherwise the simulation will not run (based on testing)
        new_msg.header.frame_id = msg->header.frame_id;
        new_msg.joint_names = msg->joint_names;
        new_msg.points = msg->points;

        // Log the republished message
        RCLCPP_INFO(this->get_logger(), "Republishing trajectory message to /joint_trajectory_controller/joint_trajectory.");

        // Publish the new message
        publisher_->publish(new_msg);

        // Log the published message
        RCLCPP_INFO(this->get_logger(), "Published trajectory message to /joint_trajectory_controller/joint_trajectory.");
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiddleManNode>());
    rclcpp::shutdown();
    return 0;
}