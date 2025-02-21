#ifndef PID_ARM_PLANNER
#define PID_ARM_PLANNER

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "pid_utils.hpp"

using std::placeholders::_1;

namespace pid_arm_planner
{
    class PidArmPlanner : public rclcpp::Node
    {
        public:
        explicit PidArmPlanner(const rclcpp::NodeOptions&option = rclcpp::NodeOptions());

        void target_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void current_callback(const std_msgs::msg::Float32::SharedPtr msg);
        
        private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
        rclcpp::Time last_;

        std::shared_ptr<PIDController> pid_;
        PIDGain gain_;
        float max_limit_;

        float current_;
    };
}

#endif