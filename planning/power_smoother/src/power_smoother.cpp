#include "power_smoother/power_smoother.hpp"

namespace power_smoother
{
    PowerSmoother::PowerSmoother(const rclcpp::NodeOptions& options) : Node("PowerSmoother", options)
    {
        gain_ = this->declare_parameter("gain", 10);
        pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/output", 0);

        target_ = nullptr;
        prev_ = std_msgs::msg::Int64MultiArray();
        prev_.data.resize(3);
        prev_.data[0] = 0;
        prev_.data[1] = 0;
        prev_.data[2] = 0;

        sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "/input",
            0,
            std::bind(&PowerSmoother::topic_callback, this, _1)
        );

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PowerSmoother::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Start %s. gain : %d", this->get_name(), gain_);
    }

    void PowerSmoother::topic_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
    {
        target_ = msg;
    }

    void PowerSmoother::timer_callback()
    {
        auto send = std_msgs::msg::Int64MultiArray();
        send.data.clear();
        send.data.resize(3);
        if(target_ != nullptr)
        {
            for(int i = 0; i < 3; i++)
            {
                int vec = target_->data[i] - prev_.data[i];
                if(vec > 0)
                {
                    if(vec > gain_)
                    {
                        send.data[i] = prev_.data[i] + gain_;
                    }
                    else
                    {
                        send.data[i] = target_->data[i];
                    }
                }
                else if(vec < 0)
                {
                    if(abs(vec) > gain_)
                    {
                        send.data[i] = prev_.data[i] - gain_;
                    }
                    else
                    {
                        send.data[i] = target_->data[i];
                    }
                }
                else
                {
                    send.data[i] = target_->data[i];
                }

                prev_.data[i] = send.data[i];
            }

            pub_->publish(send);
        }
    }
}