#include "rs_d455_ros2/rs_d455_ros2.hpp"

namespace rs_d455_ros2
{
    RealSenseD455_ROS2::RealSenseD455_ROS2(const rclcpp::NodeOptions& options) : Node("RealSenseD455_ROS2", options)
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/realsense/image", qos_settings);

        realsense_ = std::make_shared<RealSense>();

        // sleep(3);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(17), std::bind(&RealSenseD455_ROS2::timer_callback, this));

        const auto device_info = realsense_->getDeviceInfo();

        RCLCPP_INFO(this->get_logger(), "Device Name : %s", device_info.name.c_str());
        RCLCPP_INFO(this->get_logger(), "FirmWare Version : %s", device_info.firmware_version.c_str());
        RCLCPP_INFO(this->get_logger(), "Product Line : %s", device_info.product_line.c_str());
        RCLCPP_INFO(this->get_logger(), "USB Type : %s", device_info.usb_type.c_str());

        RCLCPP_INFO(this->get_logger(), "Start RealSenseD455 ROS2 !!");
    }

    void RealSenseD455_ROS2::timer_callback()
    {
        cv::Mat image;
        realsense_->getColorFrame(image);
        auto imu = realsense_->getIMU();
        // RCLCPP_INFO(this->get_logger(), "Accel: %lf, %lf, %lf", imu.acc_x, imu.acc_y, imu.acc_z);
        // RCLCPP_INFO(this->get_logger(), "Ang: %lf, %lf, %lf", imu.ang_x, imu.ang_y, imu.ang_z);

        cv_bridge::CvImage img_bridge;
        auto header = std_msgs::msg::Header();
        header.frame_id = "realsense";
        header.stamp = this->get_clock()->now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
        sensor_msgs::msg::Image img_msg;
        img_bridge.toImageMsg(img_msg);

        image_publisher_->publish(img_msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rs_d455_ros2::RealSenseD455_ROS2)