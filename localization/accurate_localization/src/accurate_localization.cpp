#include "accurate_localization/accurate_localization.hpp"

namespace accurate_localization
{
    AccurateLocalization::AccurateLocalization(const rclcpp::NodeOptions& options) : Node("AccurateLocalization", options)
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            0,
            std::bind(&AccurateLocalization::odom_callback, this, _1)
        );

        arm_frontback_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/arm/frontback",
            0,
            std::bind(&AccurateLocalization::arm_frontback_callback, this, _1)
        );

        arm_updown_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/arm/updown",
            0,
            std::bind(&AccurateLocalization::arm_updown_callback, this, _1)
        );
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 0);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 0);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&AccurateLocalization::timer_callback, this));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        updown_ = geometry_msgs::msg::TransformStamped();
        frontback_ = geometry_msgs::msg::TransformStamped();
        odom_ = geometry_msgs::msg::TransformStamped();
        path_ = nav_msgs::msg::Path();

        odom_.header.frame_id = this->declare_parameter("frame_id", "map");
        path_.header = odom_.header;
        odom_.child_frame_id = this->declare_parameter("child_frame_id", "base_link");
        updown_.header.frame_id = this->declare_parameter("arm_base_id", "arm_link");
        updown_.child_frame_id = this->declare_parameter("arm_updown_id", "updown");
        frontback_.header.frame_id = updown_.child_frame_id;
        frontback_.child_frame_id = this->declare_parameter("arm_frontback_id", "frontback");
    }

    void AccurateLocalization::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_.transform.translation.x = msg->pose.pose.position.x;
        odom_.transform.translation.y = msg->pose.pose.position.y;
        odom_.transform.translation.z = msg->pose.pose.position.z;
        odom_.transform.rotation = msg->pose.pose.orientation;
    }

    void AccurateLocalization::arm_frontback_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        frontback_.transform.translation.z = msg->data;
    }

    void AccurateLocalization::arm_updown_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        updown_.transform.translation.x = msg->data;
    }

    void AccurateLocalization::timer_callback()
    {
        geometry_msgs::msg::PoseStamped p;
        p.pose.position.x = odom_.transform.translation.x;
        p.pose.position.y = odom_.transform.translation.y;
        p.pose.position.z = odom_.transform.translation.z;
        p.pose.orientation = odom_.transform.rotation;
        p.header = odom_.header;

        pose_pub_->publish(p);
        path_.poses.push_back(p);

        tf_broadcaster_->sendTransform(odom_);
        tf_broadcaster_->sendTransform(updown_);
        tf_broadcaster_->sendTransform(frontback_);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(accurate_localization::AccurateLocalization)