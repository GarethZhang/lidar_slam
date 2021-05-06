//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_NDT_REGISTRATION_H
#define LIDAR_SLAM_NDT_REGISTRATION_H

#include "registration_method.h"
#include <pcl/registration/ndt.h>

class NDTRegistration:public RegistrationMethod{
public:
    NDTRegistration(const YAML::Node& yaml_config_node);

    void setInputTarget(PointCloudData::pointCloudTypePtr& target_cloud);

    void scanRegistration(PointCloudData::pointCloudTypePtr& source_cloud,
                          PointCloudData::pointCloudTypePtr& output_cloud,
                          Common::TMat& init_pose,
                          Common::TMat& pose);

private:
    pcl::NormalDistributionsTransform<PointCloudData::pointType, PointCloudData::pointType> ndt_;

};

#endif //LIDAR_SLAM_NDT_REGISTRATION_H
