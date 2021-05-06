//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_REGISTRATION_METHOD_H
#define LIDAR_SLAM_REGISTRATION_METHOD_H

#include "lidar_slam/sensor_data/point_cloud_data.h"

class RegistrationMethod{
public:
    virtual ~RegistrationMethod() = default;

    virtual void setInputTarget();

    virtual void scanRegistration();
};

#endif //LIDAR_SLAM_REGISTRATION_METHOD_H
