#ifndef RS_D455_ROS2_HPP_
#define RS_D455_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include "realsense_utils.hpp"

namespace rs_d455_ros2
{
    class RealSenseD455_ROS2 : public rclcpp::Node
    {
        public:
        explicit RealSenseD455_ROS2(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        void timer_callback();

        private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

        std::shared_ptr<RealSense> realsense_;
    };
}

#endif