#ifndef POSITION_GOAL_CHECKER_
#define POSITION_GOAL_CHECKER_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <accurate_msgs/msg/cycle.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

namespace position_goal_checker
{
    class PositionGoalChecker : public rclcpp::Node
    {
        public:
        explicit PositionGoalChecker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        private:
        tf2::Vector3 getEuler(const geometry_msgs::msg::Quaternion& q)
        {
            tf2::Quaternion tf2_quat;
            tf2_quat.setW(q.w);
            tf2_quat.setX(q.x);
            tf2_quat.setY(q.y);
            tf2_quat.setZ(q.z);

            tf2::Matrix3x3 mat(tf2_quat);

            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);

            return tf2::Vector3(roll, pitch, yaw);
        }
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
        rclcpp::Publisher<accurate_msgs::msg::Cycle>::SharedPtr publisher_;

        float pos_threshold_;
        float theta_threshold_;
        geometry_msgs::msg::PoseStamped::SharedPtr target_pose_;
    };
}

#endif