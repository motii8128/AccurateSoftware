#ifndef PCL_NDT_HPP_
#define PCL_NDT_HPP_

#include "types.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>

namespace motiodom
{
    class NDT
    {
        public:
        /// @brief コンストラクタ
        /// @param voxel_grid_leaf_size ダウンサンプリングする際のボクセルの大きさ
        /// @param eps NDTの収束条件。小さくすると精度が上がるが、計算にかかる時間が増える
        /// @param step_size １ステップあたりの変換量。大きすぎると収束しない可能性有り。だけど小さいと計算にかかる時間が増える。
        /// @param resolution 点群データをグリッドに分けるときの分解能。大きいと点群データの解像度が落ちる。小さいと精度は上がるが、計算に時間かかる
        /// @param max_iter 最大実行回数。基本収束するまで行うが、収束しない場合に何回実行するかを決める。
        NDT(float voxel_grid_leaf_size, float eps, float step_size, float resolution, int max_iter);

        /// @brief LidarオンリーかIMUも使うか決める
        /// @param enable trueだとLidarオンリーモード
        void setEnableOnlyLidar(bool enable);

        /// @brief マップ点群を初期化する
        /// @param ros_cloud sensor_msgs/msg/PointCloudの点群
        void initRegistraion(const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud);

        /// @brief NDTスキャンマッチングを行う
        /// @param ros_cloud sensor_msgs/msg/PointCloudの点群
        /// @param posture 姿勢のクォータニオン
        void compute(const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud, const Quat &posture);

        /// @brief 移動量を取得する
        /// @return Vec3の移動量
        Vec3 getTranslation();

        /// @brief 姿勢を取得する
        /// @return Quatのクォータニオン姿勢
        Quat getPosture();

        /// @brief ndtによってできたマップ点群を取得する
        /// @return sensor_msgs/msg/PointCloud2のROS2メッセージ
        sensor_msgs::msg::PointCloud2 getMapPointCloud();

        private:
        /// @brief ボクセルグリッドフィルターを使ってダウンサンプリングする
        /// @param pointcloud 入力点群
        void dowmSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, float voxel_grid_leafsize)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
            vg_filter.setInputCloud(pointcloud);
            vg_filter.setLeafSize(voxel_grid_leafsize, voxel_grid_leafsize,voxel_grid_leafsize);
            vg_filter.filter(*tmp);

            *pointcloud = *tmp;
        }

        void ros2pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr output, const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr new_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            for(const auto & ros_p : ros_cloud->points)
            {
                new_pcl->points.push_back(pcl::PointXYZ(ros_p.x, ros_p.y, ros_p.z));
            }

            *output = *new_pcl;
        }

        float voxel_grid_leafsize_;
        float eps_;
        float step_size_;
        float resolution_;
        int max_iter_;
        bool enable_only_lidar_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_pointcloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr translated_pointcloud_;
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_;
        Vec3 last_pose_;
        Quat last_posture_;
    };
}

#endif