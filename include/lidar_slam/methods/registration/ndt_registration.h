//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_NDT_REGISTRATION_H
#define LIDAR_SLAM_NDT_REGISTRATION_H

#include "registration_method.h"

class NDTRegistration:public RegistrationMethod{
public:
    NDTRegistration(const YAML::Node& yaml_config_node);

    void setInputTarget(PointCloudData::pointCloudTypePtr& input_cloud);

    void scanRegistration();


};

#endif //LIDAR_SLAM_NDT_REGISTRATION_H
