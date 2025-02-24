#include "position_goal_checker/position_goal_checker.hpp"

namespace position_goal_checker
{
    PositionGoalChecker::PositionGoalChecker(const rclcpp::NodeOptions& options) : Node("PositionGoalChecker", options)
    {
        pos_threshold_ = this->declare_parameter("pos_threshold", 0.1);
        theta_threshold_ = this->declare_parameter("theta_threshold", 0.1);

        current_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current",
            0,
            std::bind(&PositionGoalChecker::current_callback, this, _1)
        );

        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target",
            0,
            std::bind(&PositionGoalChecker::target_callback, this, _1)
        );

        publisher_ = this->create_publisher<accurate_msgs::msg::Cycle>("/cycle", 0);

        target_pose_ = nullptr;

        RCLCPP_INFO(this->get_logger(), "Start PositionGoalChecker");
    }

    void PositionGoalChecker::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose_ = msg;
    }

    void PositionGoalChecker::current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if(target_pose_ != nullptr)
        {
            const auto dx = target_pose_->pose.position.x - msg->pose.position.x;
            const auto dy = target_pose_->pose.position.y - msg->pose.position.y;
            const auto p2p = std::sqrt(dx*dx + dy*dy);

            const auto current_euler = getEuler(msg->pose.orientation);
            const auto target_euler = getEuler(target_pose_->pose.orientation);
            const auto dtheta = target_euler.z() - current_euler.z();

            auto cycle_msg = accurate_msgs::msg::Cycle();
            if(p2p < pos_threshold_ && dtheta < theta_threshold_)
            {
                cycle_msg.data = accurate_msgs::msg::Cycle::GOAL;
            }
            else
            {
                cycle_msg.data = accurate_msgs::msg::Cycle::PROGRESS;
            }

            publisher_->publish(cycle_msg);
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(position_goal_checker::PositionGoalChecker)