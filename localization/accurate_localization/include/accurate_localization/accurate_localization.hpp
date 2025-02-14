#ifndef ACCURATE_LOCALIZATION_HPP_
#define ACCURATE_LOCALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>

using std::placeholders::_1;

namespace accurate_localization
{
    class AccurateLocalization : public rclcpp::Node
    {
        public:
        AccurateLocalization(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void arm_updown_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void arm_frontback_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void timer_callback();
        
        private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr arm_updown_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr arm_frontback_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::TransformStamped updown_;
        geometry_msgs::msg::TransformStamped frontback_;
        geometry_msgs::msg::TransformStamped odom_;
        nav_msgs::msg::Path path_;
    };
}

#endif