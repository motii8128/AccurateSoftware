#ifndef PID_ARM_PLANNER
#define PID_ARM_PLANNER

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

namespace pid_arm_planner
{
    class PIDArmPlanner : public rclcpp::Node
    {
        public:
        explicit PIDArmPlanner(const rclcpp::NodeOptions&option = rclcpp::NodeOptions());

        void current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        
    };
}

#endif