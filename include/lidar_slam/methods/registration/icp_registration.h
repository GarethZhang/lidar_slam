//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_ICP_REGISTRATION_H
#define LIDAR_SLAM_ICP_REGISTRATION_H

#include "registration_method.h"
#include <pcl/registration/icp.h>

class ICPRegistration:public RegistrationMethod{
public:
    ICPRegistration(const YAML::Node& yaml_config_node);

    void setInputTarget(PointCloudData::pointCloudTypePtr& target_cloud);

    void scanRegistration(PointCloudData::pointCloudTypePtr& source_cloud,
                          PointCloudData::pointCloudTypePtr& output_cloud,
                          Common::TMat& init_pose,
                          Common::TMat& pose);

private:
    pcl::IterativeClosestPoint<PointCloudData::pointType , PointCloudData::pointType> icp_;

};

#endif //LIDAR_SLAM_ICP_REGISTRATION_H
