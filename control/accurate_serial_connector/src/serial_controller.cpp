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

        timer_ = this->create_wall_timer(50ms, std::bind(&SerialController::timer_callback, this));

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
        int m1, m2, m3, w1, w2, w3;
        if(wheel_cmd_  == nullptr)
        {
            w1 = 0;
            w2 = 0;
            w3 = 0;
        }
        else
        {
            w1 = wheel_cmd_->data[0];
            w2 = wheel_cmd_->data[1];
            w3 = wheel_cmd_->data[2];
        }

        if(machine_cmd_ == nullptr)
        {
            m1 = 0;
            m2 = 0;
            m3 = 0;
        }
        else
        {
            m1 = machine_cmd_->data[0] / 5;
            m2 = machine_cmd_->data[1] / 5;
            m3 = machine_cmd_->data[2] / 5;
        }

        uint8_t buf[7];
        buf[0] = w1 * 1.27 + 127;
        buf[1] = w2 * 1.27 + 127;
        buf[2] = w3 * 1.27 + 127;
        buf[3] = m1 * 1.27 + 127;
        buf[4] = m2 * 1.27 + 127;
        buf[5] = m3 * 1.27 + 127;
        buf[6] = '\n';
        for(int i = 0; i < 6; i++)
        {
            if(buf[i] > 255)
            {
                buf[i] = 255;
            }
            else if(buf[i] < 0)
            {
                buf[i] = 0;
            }
        }

        serial_->ClearBuffer();
        RCLCPP_INFO(this->get_logger(), "Write: %d,%d,%d,%d,%d,%d", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
        const auto serial_write_result = serial_->WritePort(buf);
        if(serial_write_result)
        {
            const auto read_string = serial_->ReadPort();
            // RCLCPP_INFO(this->get_logger(), "Read: %s", read_string.c_str());

            std::vector<int> values;
            std::stringstream ss(read_string);
            std::string tmp;

            while(std::getline(ss, tmp, ','))
            {
                try
                {
                    values.push_back(std::stoi(tmp));
                }
                catch(const std::exception& e)
                {
                    values.push_back(0);
                }
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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(serial_controller::SerialController)