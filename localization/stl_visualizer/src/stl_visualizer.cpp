#include "stl_visualizer/stl_visualizer.hpp"

namespace stl_visualizer
{
    StlVisualizer::StlVisualizer(const rclcpp::NodeOptions& options) : Node("StlVisualizer", options)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/robot_model", 0);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StlVisualizer::timer_callback, this));

        marker_ = visualization_msgs::msg::Marker();

        marker_.header.frame_id = this->declare_parameter("frame_id", "robot");
        marker_.ns = this->declare_parameter("namespace", "2025_bunai");
        marker_.id = 1;
        marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker_.action = visualization_msgs::msg::Marker::ADD;

        const auto scale = this->declare_parameter("scale", 1.0);
        marker_.scale.x = scale;
        marker_.scale.y = scale;
        marker_.scale.z = scale;
        
        marker_.color.r = this->declare_parameter("color.red", 1.0);
        marker_.color.g = this->declare_parameter("color.green", 1.0);
        marker_.color.b = this->declare_parameter("color.blue", 1.0);
        marker_.color.a = this->declare_parameter("color.alpha", 1.0);

        marker_.lifetime.sec = 0.0;
        marker_.lifetime.nanosec = 0.0;

        marker_.frame_locked = false;

        marker_.mesh_resource = "file://" + this->declare_parameter<std::string>("model_path", "model_path");
        marker_.mesh_use_embedded_materials = false;

        RCLCPP_INFO(this->get_logger(), "Start StlVisualizer. path = %s", marker_.mesh_resource.c_str());
    }

    void StlVisualizer::timer_callback()
    {
        marker_.header.stamp =this->get_clock()->now();
        publisher_->publish(marker_);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(stl_visualizer::StlVisualizer)