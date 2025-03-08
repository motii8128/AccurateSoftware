#include "color_board_connector/color_board_connector.hpp"

namespace color_board_connector
{
    ColorBoardConnector::ColorBoardConnector(const rclcpp::NodeOptions& options) : Node("ColorBoardConnector", options)
    {
        port_path_param_ = this->declare_parameter("port_path", "/dev/ttyACM0");
        serial_ = std::make_shared<SerialHandler>();

        auto serial_open_result = serial_->OpenPort(port_path_param_);

        sub_status_ = this->create_subscription<accurate_msgs::msg::Status>(
            "/status",
            0,
            std::bind(&ColorBoardConnector::status_callback, this, _1)
        );

        sub_arm_state_ = this->create_subscription<std_msgs::msg::Float32>(
            "/arm_state",
            0,
            std::bind(&ColorBoardConnector::arm_state_callback, this, _1)
        );

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ColorBoardConnector::timer_callback, this));

        status_ = nullptr;
        arm_cmd_ = 0.0;

        if(serial_open_result)
        {
            RCLCPP_INFO(this->get_logger(), "Open %s and start ColorBoardConnector", port_path_param_.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial : %s", port_path_param_.c_str());
            serial_->ClosePort();
        }
    }

    void ColorBoardConnector::status_callback(const accurate_msgs::msg::Status::SharedPtr msg)
    {
        status_ = msg;
    }

    void ColorBoardConnector::arm_state_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        arm_cmd_ = msg->data;
    }

    void ColorBoardConnector::timer_callback()
    {
        if(status_ != nullptr)
        {
            int arm = 0;
            if(arm_cmd_ > 0.0)
            {
                arm = 2;
            }
            else if(arm_cmd_ < 0.0)
            {
                arm = 1;
            }
            else
            {
                arm = 0;
            }
            const auto tx_packet = std::to_string(status_->data) + ',' + std::to_string(arm) + 'e';

            const auto serial_write_result = serial_->WritePort(tx_packet);
            if(!serial_write_result)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to ColorBoard.");
            }
        }
    }
}