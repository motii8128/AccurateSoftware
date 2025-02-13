#include "near_point_remover/near_point_remover.hpp"

namespace near_point_remover
{
    NearPointRemover::NearPointRemover(const rclcpp::NodeOptions& options) : Node("NearPointRemover", options)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/output", 0);

        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/input",
            0,
            std::bind(&NearPointRemover::topic_callback, this, _1)
        );

        param_range_ = this->declare_parameter("range", 0.1);

        RCLCPP_INFO(this->get_logger(), "Start %s", this->get_name());
    }

    void NearPointRemover::topic_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud result_msg;
        result_msg.header = msg->header;

        for(const auto& p : msg->points)
        {
            const auto dist = std::sqrt(p.x*p.x + p.y*p.y);
            if(dist > param_range_)
            {
                result_msg.points.push_back(p);
            }
        }

        publisher_->publish(result_msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(near_point_remover::NearPointRemover)