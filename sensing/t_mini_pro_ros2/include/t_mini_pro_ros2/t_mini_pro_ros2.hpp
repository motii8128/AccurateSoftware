#ifndef T_MINI_PRO_ROS2
#define T_MINI_PRO_ROS2

#include "ydlidar.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

namespace t_mini_pro_ros2
{
    class T_MiniProROS2 : public rclcpp::Node
    {
        public:
        explicit T_MiniProROS2(const rclcpp::NodeOptions& option=rclcpp::NodeOptions());

        void timer_callback();

        private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;

        std::shared_ptr<YDLidarDriver> ydlidar_;
        int param_baudrate_;
        bool param_reverse_;
        std::string param_frame_id_;
    };
}

#endif