#ifndef ACCURATE_MANAGER_HPP_
#define ACCURATE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <accurate_msgs/msg/status.hpp>

namespace accurate_msgs
{
    class AccurateManager : public rclcpp::Node
    {
        public:
        explicit AccurateManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
        rclcpp::Subscription<accurate_msgs::msg::Status>::SharedPtr status_subscriber_;
    };
}

#endif