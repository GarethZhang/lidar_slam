//
// Created by haowei on 2021-05-04.
//

#ifndef LIDAR_SLAM_ODOMETRY_NODE_H
#define LIDAR_SLAM_ODOMETRY_NODE_H

#include "odometry.h"

class OdometryNode{
public:
    OdometryNode(ros::NodeHandle *nodehandle);

    void runOdometry();

private:
    void getParams();
    void readDataFromSub();
    bool isDataEmpty();
    void readData();
    void updateOdometry();

    ros::NodeHandle nh_;

    std::string map_frame_, velodyne_frame_;
    std::string velodyne_topic_, odom_topic_;
    std::string yaml_config_fname_;
    float queue_size_;

    std::shared_ptr<PointCloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<Odometry> odometry_ptr_;
    std::shared_ptr<OdometryPublisher> odometry_pub_ptr_;

    std::queue<PointCloudData> queue_;

    Common::TMat T_o_s_odom_ = Common::TMat::Identity();

    PointCloudData curr_cloud_;

};

#endif //LIDAR_SLAM_ODOMETRY_NODE_H
