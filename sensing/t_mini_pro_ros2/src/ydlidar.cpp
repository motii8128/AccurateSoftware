#include "t_mini_pro_ros2/ydlidar.hpp"

namespace t_mini_pro_ros2
{
    YDLidarDriver::YDLidarDriver(int baudrate, bool reverse):baudrate_(baudrate),reverse_(reverse)
    {
        ydlidar::os_init();

        lidar_ = std::make_unique<CYdLidar>();
        scan_ = std::make_unique<LaserScan>();
    }

    bool YDLidarDriver::startLidar()
    {
        std::map<std::string, std::string> port_lists = ydlidar::lidarPortList();

        if(port_lists.empty())
        {
            return false;
        }

        bool isSingleChannel = false;
        float frequency = 10.0;

        port_ = std::string("/dev/ttyUSB0");

        lidar_->setlidaropt(LidarPropSerialPort, port_.c_str(), port_.size());

        std::string ignore_array;
        ignore_array.clear();
        lidar_->setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

        lidar_->setlidaropt(LidarPropSerialBaudrate, &baudrate_, sizeof(int));

        int optval = TYPE_TRIANGLE;
        lidar_->setlidaropt(LidarPropLidarType, &optval, sizeof(int));

        optval = YDLIDAR_TYPE_SERIAL;
        lidar_->setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

        optval = isSingleChannel ? 3 : 4;
        lidar_->setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
        optval = 4;
        lidar_->setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
        optval = 8;
        lidar_->setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

        bool b_optvalue = true;
        lidar_->setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
        b_optvalue = false;
        lidar_->setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
        lidar_->setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
        b_optvalue = true;
        lidar_->setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
        lidar_->setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
        b_optvalue = true;
        lidar_->setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
        b_optvalue = false;
        lidar_->setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
        b_optvalue = false;
        lidar_->setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

        float f_optvalue = 180.0f;
        lidar_->setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
        f_optvalue = -180.0f;
        lidar_->setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
        f_optvalue = 64.f;
        lidar_->setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
        f_optvalue = 0.05f;
        lidar_->setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
        lidar_->setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

        lidar_->enableGlassNoise(false);
        lidar_->enableSunNoise(false);

        bool ret = lidar_->initialize();
        if(!ret)
        {
            return false;
        }

        float pitch = .0f;
        if (!lidar_->getPitchAngle(pitch))
        {
            return false;
        }
        else
        {
            pitch_ = pitch;
        }

        ret = lidar_->turnOn();
        if (!ret) 
        {
            return false;
        }

        return true;
    }

    void YDLidarDriver::closeLidar()
    {
        lidar_->turnOff();
        lidar_->disconnecting();

        return;
    }

    const char* YDLidarDriver::getError()
    {
        return lidar_->DescribeError();
    }

    float YDLidarDriver::getPitchAngle()
    {
        return pitch_;
    }

    bool YDLidarDriver::Scan()
    {
        LaserScan scan;
        if(ydlidar::os_isOk())
        {
            if(lidar_->doProcessSimple(scan))
            {
                scan_ = std::make_unique<LaserScan>(std::move(scan));
                return true;
            }
            else
            {
                // lidar_.turnOff();
                // lidar_.disconnecting();
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    sensor_msgs::msg::PointCloud YDLidarDriver::getScanPoints()
    {
        float dir = 0.0;
        if(reverse_)dir=1.0;
        if(!reverse_)dir=-1.0;

        auto result = sensor_msgs::msg::PointCloud();
         
        for(const auto &point : scan_->points)
        {
            auto point_msg = geometry_msgs::msg::Point32();
            point_msg.x = point.range * std::cos(dir*point.angle);
            point_msg.y = point.range * std::sin(dir*point.angle);

            result.points.push_back(point_msg);
        }

        return result;
    }
}