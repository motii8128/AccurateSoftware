#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include "omni_utils.hpp"

using std::placeholders::_1;

namespace omni_vel_ros2
{
    class OmniVelROS2 : public rclcpp::Node
    {
        public:
        OmniVelROS2(const rclcpp::NodeOptions& option=rclcpp::NodeOptions());
        void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void posture_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

        private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr posture_subscriber;
        rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr publisher_;
        std::shared_ptr<OmniWheel<double>> omni_wheel;

        geometry_msgs::msg::Vector3 posture;

        double robot_radius_param, wheel_radius_param;
        std::array<double, 3> wheels_rad;
        bool pwm_flag;
    };
}

#endif