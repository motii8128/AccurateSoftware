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
        const auto current_time = this->get_clock()->now();
        if(last_time_fb_.nanoseconds() > 0)
        {
            const auto duration = current_time - last_time_fb_;
            const auto dt = duration.seconds();

            const auto fixed_rpm = msg->data / 36.0;
            const auto resolution_per_second = fixed_rpm / 60.0;
            auto send_msg = std_msgs::msg::Float32();
            send_msg.data = resolution_per_second * dt * frontback_gear_;

            frontback_movement_pub_->publish(send_msg);
        }
        last_time_fb_ = current_time;
    }

    void RpmMovementROS2::updown_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const auto current_time = this->get_clock()->now();
        if(last_time_ud_.nanoseconds() > 0)
        {
            const auto duration = current_time - last_time_ud_;
            const auto dt = duration.seconds();

            const auto fixed_rpm = msg->data / 36.0;
            const auto resolution_per_second = fixed_rpm / 60.0;
            auto send_msg = std_msgs::msg::Float32();
            send_msg.data = resolution_per_second * dt * updown_gear_;

            updown_movement_pub_->publish(send_msg);
        }
        last_time_fb_ = current_time;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rpm_movement_ros2::RpmMovementROS2)