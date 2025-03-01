#ifndef REALSENSE_UTILS_HPP_
#define REALSENSE_UTILS_HPP_

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

namespace rs_d455_ros2
{
    struct DeviceInfo
    {
        std::string name;
        std::string firmware_version;
        std::string usb_type;
        std::string product_line;
    };

    struct ImuData
    {
        float acc_x, acc_y, acc_z;
        float ang_x, ang_y, ang_z;
    };

    class RealSense
    {
        public:
        RealSense();
        ~RealSense();

        void getColorFrame(cv::Mat& color_image);
        DeviceInfo getDeviceInfo();
        ImuData getIMU();

        private:
        rs2::context ctx_;
        rs2::pipeline pipe_;
        rs2::device_list devices_;
        ImuData imu_data_;
        const int width_ = 640;
        const int height_ = 480;
        const int fps_ = 60;
    };

}

#endif