#include "pid_arm_planner/pid_arm_planner.hpp"

namespace pid_arm_planner
{
    PidArmPlanner::PidArmPlanner(const rclcpp::NodeOptions& options) : Node("PidArmPlanner", options)
    {
        max_limit_ = this->declare_parameter("limit", 0.1);
        gain_.p_gain = this->declare_parameter("p_gain", 1.0);
        gain_.i_gain = this->declare_parameter("i_gain", 0.03);
        gain_.d_gain = this->declare_parameter("d_gain", 0.1);
        current_ = 0.0;

        pid_ = std::make_shared<PIDController>();
        pid_->setConfig(gain_,1.0, -1.0);

        target_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target",
            0,
            std::bind(&PidArmPlanner::target_callback, this, _1)
        );

        current_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/current",
            0,
            std::bind(&PidArmPlanner::current_callback, this, _1)
        );

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/rpm", 0);
    }

    void PidArmPlanner::target_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        const auto current_time = this->now();
        if(last_.nanoseconds() > 0)
        {
            rclcpp::Duration delta_time = current_time - last_;
            const auto dt = delta_time.seconds();

            if(msg->data > max_limit_)
            {
                RCLCPP_ERROR(this->get_logger(), "The target value exceeds limit.");
            }
            else
            {
                const auto pid_result = pid_->calc(msg->data, current_, dt);

                auto rpm_msg = std_msgs::msg::Float32();
                rpm_msg.data = 500.0 * pid_result;

                publisher_->publish(rpm_msg);
            }
        }
        last_ = current_time;
    }

    void PidArmPlanner::current_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_ = msg->data;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pid_arm_planner::PidArmPlanner)