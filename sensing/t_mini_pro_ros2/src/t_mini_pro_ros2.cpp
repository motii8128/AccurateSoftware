#include "t_mini_pro_ros2/t_mini_pro_ros2.hpp"

namespace t_mini_pro_ros2
{
    T_MiniProROS2::T_MiniProROS2(const rclcpp::NodeOptions& option) : Node("T_MiniPro_ROS2")
    {
        RCLCPP_INFO(this->get_logger(), "Start Initialize YD Lidar T Mini Pro ");
        this->declare_parameter("baudrate", 230400);
        this->get_parameter("baudrate", param_baudrate_);
        this->declare_parameter("enable_reverse", false);
        this->get_parameter("enable_reverse", param_reverse_);


        RCLCPP_INFO(this->get_logger(), "Get parameter.\nbaudrate=%d\nenable_reverse=%d", param_baudrate_, param_reverse_);

        ydlidar_ = std::make_shared<YDLidarDriver>(param_baudrate_, param_reverse_);
        if(ydlidar_->startLidar())
        {
            RCLCPP_INFO(this->get_logger(), "Start YD Lidar");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to Initialize YDLidar.");
            RCLCPP_ERROR(this->get_logger(), "%s", ydlidar_->getError());
            ydlidar_->closeLidar();
        }

        RCLCPP_INFO(this->get_logger(), "Initialize ROS2 Node.");
        this->declare_parameter("pointcloud_frame_id", "map");
        this->get_parameter("pointcloud_frame_id", param_frame_id_);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/ydlidar/scan", rclcpp::SystemDefaultsQoS());
        timer_ = this->create_wall_timer(10ms, std::bind(&T_MiniProROS2::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Start T-Mini Pro ROS2. \nPointCloud frame_id = %s", param_frame_id_.c_str());
    }

    void T_MiniProROS2::timer_callback()
    {
        if(ydlidar_->Scan())
        {
            auto getPointCloud = ydlidar_->getScanPoints();
            getPointCloud.header.frame_id = param_frame_id_;

            publisher_->publish(getPointCloud);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to Scan.");
            ydlidar_->closeLidar();
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(t_mini_pro_ros2::T_MiniProROS2)