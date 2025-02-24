#ifndef HAND_PLANNER_HPP_
#define HAND_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <accurate_msgs/msg/hand_status.hpp>
#include <chrono>

using std::placeholders::_1;

namespace hand_planner
{
    class HandPlanner : public rclcpp::Node
    {
        public:
        explicit HandPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void ampare_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void motor_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ampare_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rpm_publisher_;
        rclcpp::Publisher<accurate_msgs::msg::HandStatus>::SharedPtr status_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        float ampare_limit_, hand_power_;
        float get_ampare_, get_motor_;
    };
}

#endif