//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_REGISTRATION_METHOD_H
#define LIDAR_SLAM_REGISTRATION_METHOD_H

#include <yaml-cpp/yaml.h>

#include "lidar_slam/common/common.h"
#include "lidar_slam/sensor_data/point_cloud_data.h"

class RegistrationMethod{
public:
    virtual ~RegistrationMethod() = default;

    virtual void setInputTarget(PointCloudData::pointCloudTypePtr& cloud) = 0;

    virtual void scanRegistration(PointCloudData::pointCloudTypePtr& source_cloud,
                                  PointCloudData::pointCloudTypePtr& output_cloud,
                                  Common::TMat& init_pose,
                                  Common::TMat& pose) = 0;
};

#endif //LIDAR_SLAM_REGISTRATION_METHOD_H
