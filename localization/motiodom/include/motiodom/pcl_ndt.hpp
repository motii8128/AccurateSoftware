#ifndef PCL_NDT_HPP_
#define PCL_NDT_HPP_

#include "types.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/msg/point_cloud.hpp>

namespace motiodom
{
    class NDT
    {
        public:
        NDT();

        /// @brief マップ点群を初期化する
        /// @param ros_cloud sensor_msgs/msg/PointCloudの点群
        void initRegistraion(const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud);

        /// @brief NDTスキャンマッチングを行う
        /// @param ros_cloud sensor_msgs/msg/PointCloudの点群
        /// @param posture 姿勢のクォータニオン
        void compute(const sensor_msgs::msg::PointCloud::SharedPtr ros_cloud, Quat posture);

        /// @brief 移動量を取得する
        /// @return Vec3の移動量
        Vec3 getTranslation();

        private:
        /// @brief ボクセルグリッドフィルターを使ってダウンサンプリングする
        /// @param pointcloud 入力点群
        void dowmSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
            vg_filter.setInputCloud(pointcloud);
            vg_filter.setLeafSize((float)voxel_grid_leafsize_, (float)voxel_grid_leafsize_, (float)voxel_grid_leafsize_);
            vg_filter.filter(*tmp);
            *pointcloud = *tmp;
        }

        double voxel_grid_leafsize_;
        double eps_;
        double step_size_;
        double resolution_;
        int max_iter_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_pointcloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr translated_pointcloud;
    };
}

#endif