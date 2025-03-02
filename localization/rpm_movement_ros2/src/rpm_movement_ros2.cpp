#include "rpm_movement_ros2/rpm_movement_ros2.hpp"

namespace rpm_movement_ros2
{
    RpmMovementROS2::RpmMovementROS2(const rclcpp::NodeOptions& options): Node("RpmMovementROS2", options)
    {
        frontback_gear_ = this->declare_parameter("frontback_gear", 0.1);
        updown_gear_ = this->declare_parameter("updown_gear", 0.1);

        updown_movement_pub_ = this->create_publisher<std_msgs::msg::Float32>("/updown", 0);
        frontback_movement_pub_ = this->create_publisher<std_msgs::msg::Float32>("/frontback", 0);

        updown_rpm_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/rpm/updown",
            0,
            std::bind(&RpmMovementROS2::updown_rpm_callback, this, _1)
        );

        frontback_rpm_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/rpm/frontback",
            0,
            std::bind(&RpmMovementROS2::frontback_rpm_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Start RpmMovementROS2");
    }

    void RpmMovementROS2::frontback_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const auto resolution_per_second = msg->data / 60.0;

        auto send_msg = std_msgs::msg::Float32();
        send_msg.data = resolution_per_second * frontback_gear_;
        
        frontback_movement_pub_->publish(send_msg);
    }

    void RpmMovementROS2::updown_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const auto resolution_per_second = msg->data / 60.0;

        auto send_msg = std_msgs::msg::Float32();
        send_msg.data = resolution_per_second * updown_gear_;

        updown_movement_pub_->publish(send_msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rpm_movement_ros2::RpmMovementROS2)