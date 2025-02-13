#ifndef NEAR_POINT_REMOVER_HPP_
#define NEAR_POINT_REMOVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using std::placeholders::_1;

namespace near_point_remover
{
    class NearPointRemover : public rclcpp::Node
    {
        public:
        explicit NearPointRemover(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        void topic_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

        private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr ros2pcl(const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr new_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            for(const auto & ros_p : ros_cloud->points)
            {
                new_pcl->points.push_back(pcl::PointXYZ(ros_p.x, ros_p.y, ros_p.z));
            }

            return new_pcl;
        }

        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscriber_;
        double param_range_;
    };
}

#endif