#ifndef ENCODE_ODOM_HPP_
#define ENCODE_ODOM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;

namespace encode_odom
{
    class EncodeOdometer : public rclcpp::Node
    {
        public:
        explicit EncodeOdometer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void topic_callback(const std_msgs::msg::Float32::SharedPtr msg);

        private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
        float gear_radius_;
        bool reverse_;
    };
}

#endif