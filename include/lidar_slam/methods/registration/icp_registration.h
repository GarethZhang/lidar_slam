//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_ICP_REGISTRATION_H
#define LIDAR_SLAM_ICP_REGISTRATION_H

#include "registration_method.h"

class ICPRegistration:public RegistrationMethod{
public:
    ICPRegistration(const YAML::Node& yaml_config_node);

    void setInputTarget(PointCloudData::pointCloudTypePtr& input_cloud);

    void scanRegistration();


};

#endif //LIDAR_SLAM_ICP_REGISTRATION_H