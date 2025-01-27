#include "omni_vel_ros2/component.hpp"

namespace omni_vel_ros2
{
    OmniVelROS2::OmniVelROS2(const rclcpp::NodeOptions& option) : Node("OmniVelROS2", option)
    {
        cmd_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniVelROS2::cmd_callback, this, _1)
        );

        posture_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/posture",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniVelROS2::posture_callback, this, _1)
        );

        publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/target_output", rclcpp::SystemDefaultsQoS());

        this->declare_parameter("robot_radius", 1.0);
        this->get_parameter("robot_radius", robot_radius_param);
        this->declare_parameter("wheel_radius", 0.1);
        this->get_parameter("wheel_radius", wheel_radius_param);
        this->declare_parameter("wheel_rad_1", 0.0);
        this->get_parameter("wheel_rad_1", wheels_rad[0]);
        this->declare_parameter("wheel_rad_2", 120.0);
        this->get_parameter("wheel_rad_2", wheels_rad[1]);
        this->declare_parameter("wheel_rad_3", -120.0);
        this->get_parameter("wheel_rad_3", wheels_rad[2]);
        this->declare_parameter("pwm_mode", false);
        this->get_parameter("pwm_mode", pwm_flag);

        posture.x = 0.0;
        posture.y = 0.0;
        posture.z = 0.0;

        wheels_rad[0] = wheels_rad[0] * (M_PI / 180.0);
        wheels_rad[1] = wheels_rad[1] * (M_PI / 180.0);
        wheels_rad[2] = wheels_rad[2] * (M_PI / 180.0);

        omni_wheel = std::make_shared<OmniWheel<double>>(wheel_radius_param, robot_radius_param, wheels_rad, pwm_flag);

        RCLCPP_INFO(this->get_logger(), "Start OmniVelROS2");
    }

    void OmniVelROS2::posture_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        posture.x = msg->x;
        posture.y = msg->y;
        posture.z = msg->z;
    }

    void OmniVelROS2::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto target_rpm = omni_wheel->calculation(-msg->linear.y, msg->linear.x, -msg->angular.z, posture.z);

        auto new_msg = std_msgs::msg::Int64MultiArray();
        new_msg.data.push_back(static_cast<int64_t>(target_rpm[0]));
        new_msg.data.push_back(static_cast<int64_t>(target_rpm[1]));
        new_msg.data.push_back(static_cast<int64_t>(target_rpm[2]));

        publisher_->publish(new_msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(omni_vel_ros2::OmniVelROS2)