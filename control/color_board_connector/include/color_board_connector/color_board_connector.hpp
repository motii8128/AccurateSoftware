#ifndef COLOR_BOARD_CONNECTOR_HPP_
#define COLOR_BOARD_CONNECTOR_HPP_

#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <accurate_msgs/msg/status.hpp>
#include <std_msgs/msg/float32.hpp>

#include "serial_handler.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace color_board_connector
{
    class ColorBoardConnector : public rclcpp::Node
    {
        public:
        explicit ColorBoardConnector(const rclcpp::NodeOptions& options=rclcpp::NodeOptions());

        void status_callback(const accurate_msgs::msg::Status::SharedPtr msg);
        void arm_state_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<accurate_msgs::msg::Status>::SharedPtr sub_status_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_arm_state_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string port_path_param_;

        std::shared_ptr<SerialHandler> serial_;
        accurate_msgs::msg::Status::SharedPtr status_;
        float arm_cmd_;
    };
}

#endif