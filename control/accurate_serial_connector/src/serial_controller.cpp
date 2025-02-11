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

        pub_updown_encoder_ = this->create_publisher<std_msgs::msg::Float32>("/encoder/updown", 0);
        pub_frontback_encoder_ = this->create_publisher<std_msgs::msg::Float32>("/encoder/frontback", 0);
        pub_hand_ampare_ = this->create_publisher<std_msgs::msg::Float32>("/hand_ampare", 0);

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
            const auto m2 = machine_cmd_->data[1];
            const auto m3 = machine_cmd_->data[2];
            const auto w1 = wheel_cmd_->data[0];
            const auto w2 = wheel_cmd_->data[1];
            const auto w3 = wheel_cmd_->data[2];

            std::string tx = std::to_string(m1) + ',' + std::to_string(m2) + ',' + std::to_string(m3) + ','
                + std::to_string(w1) + ',' + std::to_string(w2) + ',' + std::to_string(w3) + 'e';

            const auto serial_write_result = serial_->WritePort(tx);
            if(serial_write_result)
            {
                const auto read_string = serial_->ReadPort();

                std::vector<int> values;
                std::stringstream ss(read_string);
                std::string tmp;

                while(std::getline(ss, tmp, ','))
                {
                    values.push_back(std::stoi(tmp));
                }

                if(values.size() == 3)
                {
                    std_msgs::msg::Float32 updown, frontback, ampare;
                    updown.data = values[0];
                    frontback.data = values[1];
                    ampare.data = values[2];

                    pub_updown_encoder_->publish(updown);
                    pub_frontback_encoder_->publish(frontback);
                    pub_hand_ampare_->publish(ampare);
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write serial.");
            }

        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(serial_controller::SerialController)