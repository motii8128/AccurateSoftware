#ifndef SERIAL_CONTROLLER_HPP_
#define SERIAL_CONTROLLER_HPP_

#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

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
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_updown_encoder_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_frontback_encoder_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hand_ampare_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string port_path_param_;

        std::shared_ptr<SerialHandler> serial_;
        std_msgs::msg::Int64MultiArray::SharedPtr wheel_cmd_;
        std_msgs::msg::Int64MultiArray::SharedPtr machine_cmd_;
    };
}

#endif