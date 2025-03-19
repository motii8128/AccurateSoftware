#include <accurate_manager/accurate_manager.hpp>

namespace accurate_manager
{
    AccurateManager::AccurateManager(const rclcpp::NodeOptions& options) : Node("AccurateManager", options)
    {
        target_position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target/pose", 0);
        target_updown_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/target/updown", 0);
        target_frontback_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/target/frontback", 0);
        target_hand_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/target/hand", 0);

        obj_ = nullptr;
        position_cycle_ = accurate_msgs::msg::Cycle();
        position_cycle_.data = accurate_msgs::msg::Cycle::GOAL;
        updown_status_ = accurate_msgs::msg::HandStatus();
        updown_status_.data = accurate_msgs::msg::HandStatus::EMIT;
        frontback_status_ = accurate_msgs::msg::HandStatus();
        frontback_status_.data = accurate_msgs::msg::HandStatus::PROGRESS;
        hand_status_ = accurate_msgs::msg::HandStatus();
        hand_status_.data = accurate_msgs::msg::HandStatus::PROGRESS;

        position_check_subscriber_ = this->create_subscription<accurate_msgs::msg::Cycle>(
            "/status/position",
            0,
            std::bind(&AccurateManager::position_check_callback, this, _1)
        );

        updown_check_subscriber_ = this->create_subscription<accurate_msgs::msg::HandStatus>(
            "/status/updown",
            0,
            std::bind(&AccurateManager::updown_check_callback, this, _1)
        );
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(accurate_manager::AccurateManager)