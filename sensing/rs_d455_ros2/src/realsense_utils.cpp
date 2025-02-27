#include "rs_d455_ros2/realsense_utils.hpp"

namespace rs_d455_ros2
{
    RealSense::RealSense()
    {
        devices_ = ctx_.query_devices();
        if(devices_.size())
        {
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
            cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
            pipe_.start(cfg);
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
        rs2::frameset frames = pipe_.wait_for_frames();

        rs2::video_frame color_frame = frames.get_color_frame();

        cv::Mat image(cv::Size(width_, height_), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        image.copyTo(color_image);
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
}