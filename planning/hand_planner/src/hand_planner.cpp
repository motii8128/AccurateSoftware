#include "hand_planner/hand_planner.hpp"

namespace hand_planner
{
    HandPlanner::HandPlanner(const rclcpp::NodeOptions& options) : Node("HandPlanner", options)
    {
        hand_power_ = this->declare_parameter<float>("hand_power", 500.0);
        ampare_limit_ = this->declare_parameter<float>("ampare_limist", 10.0);
        get_ampare_ = 0.0;
        get_motor_ = 0.0;

        rpm_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/rpm", 0);
        status_publisher_ = this->create_publisher<accurate_msgs::msg::Status>("/hand_status", 0);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&HandPlanner::timer_callback, this));

        ampare_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ampare",
            0,
            std::bind(&HandPlanner::ampare_callback, this, _1)
        );

        motor_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor",
            0,
            std::bind(&HandPlanner::motor_callback, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Start HandPlanner");
    }

    void HandPlanner::ampare_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        get_ampare_ = msg->data;
    }

    void HandPlanner::motor_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        get_motor_ = msg->data;
    }

    void HandPlanner::timer_callback()
    {
        auto send_msg = std_msgs::msg::Float32();
        auto status_msg = accurate_msgs::msg::Status();

        if(get_motor_ > 0.0)
        {
            if(abs(get_ampare_) > ampare_limit_)
            {
                send_msg.data = 0.0;
            }
            else
            {
                status_msg.data = accurate_msgs::msg::Status::GO_START;
                send_msg.data = hand_power_;
            }
        }
        else if(get_motor_ < 0.0)
        {
            if(abs(get_ampare_) > ampare_limit_)
            {
                send_msg.data = 0.0;
            }
            else
            {
                send_msg.data = -1.0 * hand_power_;
            }
        }
        else
        {
            send_msg.data = 0.0;
        }

        rpm_publisher_->publish(send_msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hand_planner::HandPlanner)