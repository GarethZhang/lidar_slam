//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_FILTER_METHODS_H
#define LIDAR_SLAM_FILTER_METHODS_H

#include <yaml-cpp/yaml.h>
#include "glog/logging.h"

#include "lidar_slam/sensor_data/point_cloud_data.h"

class FilterMethod{
public:
    virtual ~FilterMethod() = default;

    virtual void filter(PointCloudData::pointCloudTypePtr& input_cloud, PointCloudData::pointCloudTypePtr& output_cloud) = 0;
};

#endif //LIDAR_SLAM_FILTER_METHODS_H
