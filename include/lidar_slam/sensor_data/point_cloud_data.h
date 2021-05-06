//
// Created by haowei on 2021-05-04.
//

#ifndef LIDAR_SLAM_POINT_CLOUD_DATA_H
#define LIDAR_SLAM_POINT_CLOUD_DATA_H

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

class PointCloudData{
public:
    using pointType = pcl::PointXYZ;
    using pointCloudType = pcl::PointCloud<pointType>;
    using pointCloudTypePtr = pointCloudType::Ptr;

    PointCloudData():cloud_ptr(new pointCloudType()){
    }

    double time;
    pointCloudTypePtr cloud_ptr;
};


#endif //LIDAR_SLAM_POINT_CLOUD_DATA_H
