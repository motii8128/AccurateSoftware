#ifndef MOTISLAM_YDLIDAR_HPP_
#define MOTISLAM_YDLIDAR_HPP_

#include <CYdLidar.h>
#include <core/common/ydlidar_help.h>
#include <string>
#include <map>
#include <math.h>

#include <sensor_msgs/msg/point_cloud.hpp>

namespace t_mini_pro_ros2
{
    class YDLidarDriver
    {
        public:
        /// @brief コンストラクタ
        /// @param baudrate シリアル通信速度
        /// @param reverse trueでLidarを逆さまにした場合で計算を行う
        YDLidarDriver(int baudrate=230400, bool reverse = false);

        /// @brief セットアップ
        /// @return 実行結果（trueなら成功、falseなら失敗）
        bool startLidar();

        /// @brief Lidarをシャットダウンする
        /// @return 返り値なし
        void closeLidar();

        /// @brief エラー文を取得するパッケージ
        /// @return エラー文の文字列
        const char* getError();

        /// @brief スキャン範囲の角度を取得する
        /// @return 角度[rad]
        float getPitchAngle();

        /// @brief スキャンを実行する
        /// @return 実行結果（trueなら成功、falseなら失敗）
        bool Scan();

        /// @brief 点群を取得する
        /// @return ２次元点群
        sensor_msgs::msg::PointCloud getScanPoints();

        private:
        std::unique_ptr<CYdLidar> lidar_;
        std::unique_ptr<LaserScan> scan_;
        std::string port_;
        int baudrate_;
        float pitch_;
        bool reverse_;
    };
}

#endif