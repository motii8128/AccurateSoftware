#include "accurate_serial_connector/serial_controller.hpp"

namespace serial_controller
{
    SerialController::SerialController(const rclcpp::NodeOptions option) : Node("AccurateSerialController", option)
    {
        sub_wheel_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "/wheel", 
            0, 
            std::bind(&SerialController::wheel_callback, this, _1));

        sub_machine_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
            "/machine",
            0,
            std::bind(&SerialController::machine_callback, this, _1)
        );

        timer_ = this->create_wall_timer(10ms, std::bind(&SerialController::timer_callback, this));

        this->declare_parameter("port_path", "/dev/ttyACM0");
        this->get_parameter("port_path", port_path_param_);
        wheel_cmd_ = nullptr;
        machine_cmd_ = nullptr;

        serial_ = std::make_shared<SerialHandler>();

        auto serial_open_result = serial_->OpenPort(port_path_param_);

        if(serial_open_result)
        {
            RCLCPP_INFO(this->get_logger(), "Start AccurateSerialController");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial : %s", port_path_param_.c_str());
            serial_->ClosePort();
        }
        
    }

    void SerialController::wheel_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
    {
        wheel_cmd_ = msg;
    }

    void SerialController::machine_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
    {
        machine_cmd_ = msg;
    }

    void SerialController::timer_callback()
    {
        if(wheel_cmd_ != nullptr && machine_cmd_ != nullptr)
        {
            const auto m1 = machine_cmd_->data[0];
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(serial_controller::SerialController)