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
#include "lidar_slam/publisher/point_cloud_publisher.h"
#include "lidar_slam/subscriber/point_cloud_subscriber.h"
#include "lidar_slam/tf_broadcaster/tf_broadcaster.h"
#include "lidar_slam/methods/registration/ndt_registration.h"
#include "lidar_slam/methods/registration/icp_registration.h"
#include "lidar_slam/methods/filter/voxel_filter.h"

class Odometry{
public:
    struct Frame{
        PointCloudData cloud;
        Common::TMat T_o_s;
    };

    Odometry(ros::NodeHandle& nh, std::string& yaml_config_fname);

    void updateOdometry(PointCloudData& cloud,
                        Common::TMat& T_o_s_odom);

private:
    void setConfigs(std::string& yaml_config_fname);
    void setPublishConfigs();
    void setFilter(YAML::Node& yaml_config_node);
    void setScanRegistration(YAML::Node& yaml_config_node);
    void predictSm1S();
    void updateSubmap(Frame& cloud);

    ros::NodeHandle nh_;

    Frame curr_frame_;

    PointCloudData::pointCloudTypePtr curr_cloud_;

    std::shared_ptr<PointCloudPublisher> submap_pub_ptr_;
    std::shared_ptr<PointCloudPublisher> filter_cloud_pub_ptr_;
    std::shared_ptr<TFBroadcaster> tf_broadcaster_ptr_;

    std::deque<Frame> submaps_;

    bool first_scan_;

    size_t window_size_;

    float submap_create_dist_, dist_travel_since_lsm_;

    std::string registration_method_, filter_method_;
    std::string submap_topic_, filter_cloud_topic_, submap_frame_, filter_cloud_frame_;

    std::shared_ptr<RegistrationMethod> registration_ptr_;
    std::shared_ptr<FilterMethod> filter_ptr_, submap_filter_ptr_;

    Common::TMat T_o_s_pred_, T_o_sm1_, T_sm1_s_, T_o_lsm_;

};

#endif //LIDAR_SLAM_ODOMETRY_H
