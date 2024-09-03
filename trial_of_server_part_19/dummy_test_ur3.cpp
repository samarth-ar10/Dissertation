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
