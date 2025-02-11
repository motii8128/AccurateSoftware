#include "localization_sim/localization_sim.hpp"

namespace localization_sim
{
    LocalizationSim::LocalizationSim(const rclcpp::NodeOptions& options) : Node("LocalizationSim", options)
    {
        pose_ = geometry_msgs::msg::TransformStamped();
        pose_.header.frame_id = this->declare_parameter("frame_id", "map");
        pose_.child_frame_id = this->declare_parameter("child_frame_id", "base_link");
        yaw_ = 0.0;

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            0,
            std::bind(&LocalizationSim::topic_callback, this, _1)
        );

        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 0);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LocalizationSim::timer_callback, this));
    }

    void LocalizationSim::topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        yaw_ += msg->angular.z * 0.05;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);

        pose_.transform.translation.x += msg->linear.x * 0.05;
        pose_.transform.translation.y += msg->linear.y * 0.05;
        pose_.transform.translation.z = 0.0;

        pose_.transform.rotation.w = q.w();
        pose_.transform.rotation.x = q.x();
        pose_.transform.rotation.y = q.y();
        pose_.transform.rotation.z = q.z();
    }

    void LocalizationSim::timer_callback()
    {
        tf_broadcaster_->sendTransform(pose_);
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = pose_.child_frame_id;
        p.pose.position.x = pose_.transform.translation.x;
        p.pose.position.y = pose_.transform.translation.y;
        p.pose.position.z = pose_.transform.translation.z;

        p.pose.orientation = pose_.transform.rotation;

        pub_->publish(p);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(localization_sim::LocalizationSim)