#include "stl_visualizer/stl_visualizer.hpp"

namespace stl_visualizer
{
    StlVisualizer::StlVisualizer(const rclcpp::NodeOptions& options) : Node("StlVisualizer", options)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/robot_model", 0);
    }
}