#ifndef ACCURATE_LOCALIZATION_HPP_
#define ACCURATE_LOCALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace accurate_localization
{
    class AccurateLocalization : public rclcpp::Node
    {
        public:
        AccurateLocalization(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        
        private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_sub_;
    };
}

#endif