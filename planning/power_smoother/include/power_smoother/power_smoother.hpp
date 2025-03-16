#ifndef POWER_SMOOTHER_HPP_
#define POWER_SMOOTHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <chrono>

using std::placeholders::_1;

namespace power_smoother
{
    class PowerSmoother : public rclcpp::Node
    {
        public:
        explicit PowerSmoother(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void topic_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_;
        rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std_msgs::msg::Int64MultiArray::SharedPtr target_;
        std_msgs::msg::Int64MultiArray prev_;
        int gain_;
    };
}

#endif