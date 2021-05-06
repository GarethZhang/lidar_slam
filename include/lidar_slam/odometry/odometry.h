//
// Created by haowei on 2021-05-04.
//

#ifndef LIDAR_SLAM_ODOMETRY_H
#define LIDAR_SLAM_ODOMETRY_H


#include <yaml-cpp/yaml.h>
#include "glog/logging.h"

#include "pointmatcher/PointMatcher.h"

#include "lidar_slam/common/common.h"
#include "lidar_slam/publisher/odometry_publisher.h"
#include "lidar_slam/subscriber/point_cloud_subscriber.h"
#include "lidar_slam/methods/registration/ndt_registration.h"
#include "lidar_slam/methods/registration/icp_registration.h"
#include "lidar_slam/methods/filter/voxel_filter.h"

class Odometry{
public:
    struct Frame{
        PointCloudData cloud;
        Common::TMat T_o_s;
    };

    Odometry(std::string& yaml_config_fname);

    void updateOdometry(PointCloudData::pointCloudTypePtr& cloud,
                        Common::TMat& init_pose);

private:
    void setConfigs(std::string& yaml_config_fname);
    void setFilter(YAML::Node& yaml_config_node);
    void setScanRegistration(YAML::Node& yaml_config_node);

    Frame curr_frame_;

    bool first_scan_;

    std::string registration_method_, filter_method_;

    std::shared_ptr<RegistrationMethod> registration_ptr_;
    std::shared_ptr<FilterMethod> filter_ptr_;

};

#endif //LIDAR_SLAM_ODOMETRY_H
