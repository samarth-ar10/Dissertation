#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class UR3eController : public rclcpp::Node
{
public:
    UR3eController() : Node("ur3e_controller")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&UR3eController::publish_trajectory, this));
    }

private:
    void publish_trajectory()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.header.stamp = this->now();
        message.joint_names = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"};

        // First point
        auto point1 = trajectory_msgs::msg::JointTrajectoryPoint();
        point1.positions = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0};
        point1.time_from_start = rclcpp::Duration(2, 0); // 2 seconds

        // Second point
        auto point2 = trajectory_msgs::msg::JointTrajectoryPoint();
        point2.positions = {0.5, -1.0, 1.0, 0.5, 1.0, 0.5};
        point2.time_from_start = rclcpp::Duration(4, 0); // 4 seconds

        // Third point
        auto point3 = trajectory_msgs::msg::JointTrajectoryPoint();
        point3.positions = {1.0, -0.5, 0.5, 1.0, 0.5, 1.0};
        point3.time_from_start = rclcpp::Duration(6, 0); // 6 seconds

        message.points.push_back(point1);
        message.points.push_back(point2);
        message.points.push_back(point3);

        RCLCPP_INFO(this->get_logger(), "Publishing trajectory");
        publisher_->publish(message);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UR3eController>());
    rclcpp::shutdown();
    return 0;
}