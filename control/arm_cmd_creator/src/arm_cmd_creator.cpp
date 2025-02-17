#include "arm_cmd_creator/arm_cmd_creator.hpp"

namespace arm_cmd_creator
{
    ArmCmdCreator::ArmCmdCreator(const rclcpp::NodeOptions& options) : Node("ArmCmdCreator", options)
    {
        sub1_ = this->create_subscription<std_msgs::msg::Float32>(
            "/arm/frontback",
            0,
            std::bind(&ArmCmdCreator::arm1_callback, this, _1)
        );
        sub2_ = this->create_subscription<std_msgs::msg::Float32>(
            "/arm/updown",
            0,
            std::bind(&ArmCmdCreator::arm2_callback, this, _1)
        );
        sub3_ = this->create_subscription<std_msgs::msg::Float32>(
            "/arm/hand",
            0,
            std::bind(&ArmCmdCreator::arm3_callback, this, _1)
        );

        pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/arm_cmd", 0);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ArmCmdCreator::timer_callback, this));

        frontback_ = std_msgs::msg::Float32();
        updown_ = std_msgs::msg::Float32();
        hand_ = std_msgs::msg::Float32();

        RCLCPP_INFO(this->get_logger(), "Start ArmCmdCreator.");
    }

    void ArmCmdCreator::arm1_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        frontback_.data = msg->data;
    }

    void ArmCmdCreator::arm2_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        updown_.data = msg->data;
    }

    void ArmCmdCreator::arm3_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        hand_.data = msg->data;
    }

    void ArmCmdCreator::timer_callback()
    {
        auto send_msg = std_msgs::msg::Int64MultiArray();
        send_msg.data.push_back(frontback_.data);
        send_msg.data.push_back(updown_.data);
        send_msg.data.push_back(hand_.data);
        

        pub_->publish(send_msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(arm_cmd_creator::ArmCmdCreator)