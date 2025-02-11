#ifndef LOCALIZATION_SIM_HPP_
#define LOCALIZATION_SIM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>

using std::placeholders::_1;


namespace localization_sim
{
    class LocalizationSim : public rclcpp::Node
    {
        public:
        explicit LocalizationSim(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::TransformStamped pose_;
        float yaw_;
    };
}

#endif