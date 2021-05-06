//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_FILTER_METHODS_H
#define LIDAR_SLAM_FILTER_METHODS_H

#include "lidar_slam/sensor_data/point_cloud_data.h"

class FilterMethod{
    FilterMethod();

    virtual void filter(PointCloudData::cloud_ptr& input_cloud, PointCloudData::cloud_ptr& output_cloud);
};

#endif //LIDAR_SLAM_FILTER_METHODS_H
