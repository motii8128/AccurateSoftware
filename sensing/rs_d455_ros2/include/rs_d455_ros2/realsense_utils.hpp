#ifndef REALSENSE_UTILS_HPP_
#define REALSENSE_UTILS_HPP_

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

namespace rs_d455_ros2
{
    class RealSense
    {
        public:
        RealSense();
        ~RealSense();

        void getColorFrame(cv::Mat& color_image);

        private:
        rs2::pipeline pipe_;
    };
}

#endif