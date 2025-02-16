#include "encode_odom/encode_odom.hpp"

namespace encode_odom
{
    EncodeOdometer::EncodeOdometer(const rclcpp::NodeOptions& options) : Node("EncodeOdometer", options)
    {
        gear_radius_ = this->declare_parameter("gear_radius", 0.3);
        reverse_ = this->declare_parameter("enable_reverse", false);

        subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/encode",
            0,
            std::bind(&EncodeOdometer::topic_callback, this, _1)
        );

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/movement", 0);
    }

    void EncodeOdometer::topic_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        auto movement = std_msgs::msg::Float32();
        
        const auto theta = msg->data * (M_PI / 180.0);

        movement.data = gear_radius_ * 2.0 * theta;

        publisher_->publish(movement);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(encode_odom::EncodeOdometer)