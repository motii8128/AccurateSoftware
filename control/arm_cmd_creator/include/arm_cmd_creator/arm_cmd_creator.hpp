#ifndef ARM_CMD_CREATOR_HPP_
#define ARM_CMD_CREATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <chrono>

using std::placeholders::_1;

namespace arm_cmd_creator
{
    class ArmCmdCreator : public rclcpp::Node
    {
        public:
        explicit ArmCmdCreator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void arm1_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void arm2_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void arm3_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub1_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub2_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub3_;
        rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std_msgs::msg::Float32 frontback_;
        std_msgs::msg::Float32 updown_;
        std_msgs::msg::Float32 hand_;
    };
}

#endif