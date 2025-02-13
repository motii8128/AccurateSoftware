#ifndef NEAR_POINT_REMOVER_HPP_
#define NEAR_POINT_REMOVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

using std::placeholders::_1;

namespace near_point_remover
{
    class NearPointRemover : public rclcpp::Node
    {
        public:
        explicit NearPointRemover(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        void topic_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

        private:
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscriber_;
        double param_range_;
    };
}

#endif