#ifndef RPM_MOVEMENT_ROS2_HPP_
#define RPM_MOVEMENT_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;

namespace rpm_movement_ros2
{
    class RpmMovementROS2 : public rclcpp::Node
    {
        public:
        explicit RpmMovementROS2(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void updown_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void frontback_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg);

        private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr updown_rpm_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr frontback_rpm_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr updown_movement_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr frontback_movement_pub_;

        float updown_gear_, frontback_gear_;
    };
}

#endif