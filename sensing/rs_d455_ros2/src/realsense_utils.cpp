#include "rs_d455_ros2/realsense_utils.hpp"

namespace rs_d455_ros2
{
    RealSense::RealSense()
    {
        devices_ = ctx_.query_devices();
        if(devices_.size())
        {
            try
            {
                bool is_gyro, is_acc;
                is_acc = false;
                is_gyro = false;
                for (auto sensor : devices_[0].query_sensors())
                {
                    for (auto profile : sensor.get_stream_profiles())
                    {
                        if (profile.stream_type() == RS2_STREAM_GYRO)
                        {
                            is_gyro = true;
                        }
                        if (profile.stream_type() == RS2_STREAM_ACCEL)
                        {
                            is_acc = true;
                        }
                    }
                }
                if(!is_acc && !is_gyro)
                {
                    throw std::runtime_error("IMU is not supported.");
                }
                
                pipe_ = rs2::pipeline(ctx_);
                rs2::config cfg;
                cfg.disable_all_streams();
                cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
                // cfg.enable_stream(RS2_STREAM_DEPTH);
                // cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 63);
                // cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
                cfg.enable_device(std::string(devices_[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)));

                pipe_.start(cfg);
            }
            catch (const rs2::error& e) {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << std::endl;
            }
        }
        else
        {
            throw std::runtime_error("No camera is connected.");
        }
    }

    RealSense::~RealSense()
    {
        pipe_.stop();
    }

    void RealSense::getColorFrame(cv::Mat& color_image)
    {
        rs2::frameset frames = pipe_.wait_for_frames(1000);

        for(const auto& frame : frames)
        {
            if(frame.get_profile().stream_type() == RS2_STREAM_COLOR)
            {
                auto color_frame = frame.as<rs2::video_frame>();
                cv::Mat image(cv::Size(width_, height_), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                image.copyTo(color_image);
            }
            else if(frame.get_profile().stream_type() == RS2_STREAM_GYRO)
            {
                auto motion = frame.as<rs2::motion_frame>();
                rs2_vector gyro = motion.get_motion_data();
                imu_data_.ang_x = gyro.x;
                imu_data_.ang_y = gyro.y;
                imu_data_.ang_z = gyro.z;
            }
            else if(frame.get_profile().stream_type() == RS2_STREAM_ACCEL)
            {
                auto motion = frame.as<rs2::motion_frame>();
                rs2_vector acc = motion.get_motion_data();
                imu_data_.acc_x = acc.x;
                imu_data_.acc_y = acc.y;
                imu_data_.acc_z = acc.z;
            }
        }   
    }

    DeviceInfo RealSense::getDeviceInfo()
    {
        rs2::device dev = devices_[0];

        DeviceInfo dev_info;
        dev_info.name = std::string(dev.get_info(RS2_CAMERA_INFO_NAME));
        dev_info.firmware_version = std::string(dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION));
        dev_info.product_line = std::string(dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE));
        dev_info.usb_type = std::string(dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));

        return dev_info;
    }
    ImuData RealSense::getIMU()
    {
        return imu_data_;
    }
}