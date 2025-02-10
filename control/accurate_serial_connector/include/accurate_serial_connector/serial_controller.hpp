#ifndef SERIAL_CONTROLLER_HPP_
#define SERIAL_CONTROLLER_HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include "serial_handler.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace serial_controller
{
    class SerialController : public rclcpp::Node
    {
        public:
        explicit SerialController(const rclcpp::NodeOptions option=rclcpp::NodeOptions());

        void wheel_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
        void machine_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_wheel_;
        rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_machine_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string port_path_param_;

        std::shared_ptr<SerialHandler> serial_;
        std_msgs::msg::Int64MultiArray::SharedPtr wheel_cmd_;
        std_msgs::msg::Int64MultiArray::SharedPtr machine_cmd_;
    };
}

#endif