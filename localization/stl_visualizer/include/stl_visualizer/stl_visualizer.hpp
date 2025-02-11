#ifndef STL_VISUALIZER_HPP_
#define STL_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>

namespace stl_visualizer
{
    class StlVisualizer : public rclcpp::Node
    {
        public:
        explicit StlVisualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void timer_callback();

        private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        visualization_msgs::msg::Marker marker_;
    };
}

#endif