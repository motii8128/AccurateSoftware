#include "rs_d455_ros2/realsense_utils.hpp"

namespace rs_d455_ros2
{
    RealSense::RealSense()
    {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        pipe_.start(cfg);
    }

    RealSense::~RealSense()
    {
        pipe_.stop();
    }

    void RealSense::getColorFrame(cv::Mat& color_image)
    {
        rs2::frameset frames = pipe_.wait_for_frames();
    }
}