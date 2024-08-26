#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class DummyNode : public rclcpp::Node
{
public:
    DummyNode() : Node("dummy_node")
    {
        // Publisher to the middle man's input trajectory topic
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/input_joint_trajectory", 10);

        // Timer to publish the trajectory after a short delay
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&DummyNode::publish_trajectory, this));
    }

private:
    void publish_trajectory()
    {
        // Create a JointTrajectory message
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.header.stamp = this->now();
        msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        // Create multiple JointTrajectoryPoints and add them to the message
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

        trajectory_msgs::msg::JointTrajectoryPoint point1;
        point1.positions = {0.5, -0.5, 0.5, -0.5, 0.5, -0.5};  // Example positions
        point1.time_from_start.sec = 2;  // Set the duration of the movement
        points.push_back(point1);

        trajectory_msgs::msg::JointTrajectoryPoint point2;
        point2.positions = {-0.5, 0.5, -0.5, 0.5, -0.5, 0.5};  // Example positions
        point2.time_from_start.sec = 4;  // Set the duration of the movement
        points.push_back(point2);

        trajectory_msgs::msg::JointTrajectoryPoint point3;
        point3.positions = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};  // Example positions
        point3.time_from_start.sec = 6;  // Set the duration of the movement
        points.push_back(point3);

        trajectory_msgs::msg::JointTrajectoryPoint point4;
        point4.positions = {-1.0, 1.0, -1.0, 1.0, -1.0, 1.0};  // Example positions
        point4.time_from_start.sec = 8;  // Set the duration of the movement
        points.push_back(point4);

        msg.points = points;

        // Publish the message
        publisher_->publish(msg);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyNode>());
    rclcpp::shutdown();
    return 0;
}