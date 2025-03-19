#ifndef ACCURATE_MANAGER_HPP_
#define ACCURATE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <accurate_msgs/msg/status.hpp>
#include <accurate_msgs/msg/hand_status.hpp>
#include <accurate_msgs/msg/cycle.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
using std::placeholders::_1;

namespace accurate_manager
{
    class AccurateManager : public rclcpp::Node
    {
        public:
        explicit AccurateManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void position_check_callback(const accurate_msgs::msg::Cycle::SharedPtr msg);
        void updown_check_callback(const accurate_msgs::msg::HandStatus::SharedPtr msg);
        void frontback_check_callback(const accurate_msgs::msg::HandStatus::SharedPtr msg);
        void hand_check_callback(const accurate_msgs::msg::HandStatus::SharedPtr msg);
        void obj_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<accurate_msgs::msg::Cycle>::SharedPtr position_check_subscriber_;
        rclcpp::Subscription<accurate_msgs::msg::HandStatus>::SharedPtr updown_check_subscriber_;
        rclcpp::Subscription<accurate_msgs::msg::HandStatus>::SharedPtr frontback_check_subscriber_;
        rclcpp::Subscription<accurate_msgs::msg::HandStatus>::SharedPtr hand_status_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_subscriber_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_position_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_updown_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_frontback_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_hand_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

        accurate_msgs::msg::Cycle position_cycle_;
        accurate_msgs::msg::HandStatus updown_status_;
        accurate_msgs::msg::HandStatus frontback_status_;
        accurate_msgs::msg::HandStatus hand_status_;
        geometry_msgs::msg::PoseStamped::SharedPtr obj_;
    };
}

#endif