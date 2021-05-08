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

    void updateOdometry(PointCloudData& cloud,
                        Common::TMat& T_o_s_odom);

private:
    void setConfigs(std::string& yaml_config_fname);
    void setFilter(YAML::Node& yaml_config_node);
    void setScanRegistration(YAML::Node& yaml_config_node);
    void predictSm1S();
    void updateSubmap(Frame& cloud);
    double getTravelDist(Common::TMat& T_o_sm1, Common::TMat& T_o_s);

    Frame curr_frame_;

    PointCloudData::pointCloudTypePtr curr_cloud_;

    std::deque<Frame> submaps_;

    bool first_scan_;

    size_t window_size_;

    float submap_create_dist_;

    std::string registration_method_, filter_method_;

    std::shared_ptr<RegistrationMethod> registration_ptr_;
    std::shared_ptr<FilterMethod> filter_ptr_;

    Common::TMat T_o_s_pred_, T_o_sm1_, T_sm1_s_, T_o_lsm_;

};

#endif //LIDAR_SLAM_ODOMETRY_H
