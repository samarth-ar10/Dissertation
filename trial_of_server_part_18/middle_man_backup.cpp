#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <set>
#include <mutex>

class MiddleManNode : public rclcpp::Node
{
public:
    MiddleManNode()
        : Node("middle_man_node"), node_name_(""), control_node_("")
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
            "/node_name_broadcast", 10, std::bind(&MiddleManNode::name_callback, this, std::placeholders::_1));

        // Subscription to the topic where commands are published
        subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/input_joint_trajectory", 10,
            std::bind(&MiddleManNode::trajectory_callback, this, std::placeholders::_1));

        // Timer to broadcast the node's name periodically
        name_broadcast_timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&MiddleManNode::broadcast_name, this));

        RCLCPP_INFO(this->get_logger(), "MiddleManNode has been started with name: %s", node_name_.c_str());
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

        // Create a new message for the simulation
        auto sim_msg = *msg;              // Copy the incoming message
        sim_msg.header.stamp.sec = 0;     // Set sec to 0 for the simulation
        sim_msg.header.stamp.nanosec = 0; // Set nanosec to 0 for the simulation

        // Create a new message for the real robot
        auto real_msg = *msg;              // Copy the incoming message
        real_msg.header.stamp.sec = 0;     // Set sec to 0 for the real robot
        real_msg.header.stamp.nanosec = 0; // Set nanosec to 0 for the real robot

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
    rclcpp::spin(std::make_shared<MiddleManNode>());
    rclcpp::shutdown();
    return 0;
}
