#include "dualsense_ros2/dualsense_ros2_node.hpp"

namespace dualsense_ros2
{
    DualSenseROS2::DualSenseROS2(const rclcpp::NodeOptions & node_options) : rclcpp::Node("DualSenseROS2", node_options)
    {
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/dualsense/twist", 0);

        f_publisher_1 = this->create_publisher<std_msgs::msg::Float32>("/dualsense/up_and_down", 0);
        f_publisher_2 = this->create_publisher<std_msgs::msg::Float32>("/dualsense/l_sholder", 0);
        f_publisher_3 = this->create_publisher<std_msgs::msg::Float32>("/dualsense/r_sholder", 0);

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::SystemDefaultsQoS(), std::bind(&DualSenseROS2::topic_callback, this, _1));

        this->declare_parameter("max_pow", 1.0);
        this->get_parameter("max_pow", max_pow_);

        RCLCPP_INFO(this->get_logger(), "DualSenseROS2 intialize OK!!");
    }

    void DualSenseROS2::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = -1.0 * msg->axes[0];
        cmd.linear.y = msg->axes[1];
        cmd.angular.z = msg->axes[3];

        auto f1_msg = std_msgs::msg::Float32();
        auto f2_msg = std_msgs::msg::Float32();
        auto f3_msg = std_msgs::msg::Float32();

        f1_msg.data = msg->axes[7];
        f2_msg.data = msg->buttons[4] - msg->buttons[6];
        f3_msg.data = msg->buttons[5] - msg->buttons[7];

        twist_publisher_->publish(cmd);
        f_publisher_1->publish(f1_msg);
        f_publisher_2->publish(f2_msg);
        f_publisher_3->publish(f3_msg);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dualsense_ros2::DualSenseROS2)