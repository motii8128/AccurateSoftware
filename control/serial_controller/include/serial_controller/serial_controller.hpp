#ifndef SERIAL_CONTROLLER_HPP_
#define SERIAL_CONTROLLER_HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include "serial_controller/serial_handler.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace serial_controller
{
    class SerialController : public rclcpp::Node
    {
        public:
        explicit SerialController(const rclcpp::NodeOptions option=rclcpp::NodeOptions());

        void topic_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string port_path_param;

        std::shared_ptr<SerialHandler> serial;
        bool str_flag;
        std::string tx;
    };
}

#endif