//
// Created by haowei on 2021-05-04.
//

#ifndef LIDAR_SLAM_ODOMETRY_NODE_H
#define LIDAR_SLAM_ODOMETRY_NODE_H

#include "odometry.h"

class OdometryNode{
public:
    OdometryNode(ros::NodeHandle *nodehandle);

private:
    void getParams();
    bool readData();
    bool isDataEmpty();
    bool isValidData();

    ros::NodeHandle nh_;

    std::string velodyne_topic_, yaml_config_fname_;
    float queue_size_;

    std::shared_ptr<PointCloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<Odometry> odometry_ptr_;

    std::queue<PointCloudData> queue_;

};

#endif //LIDAR_SLAM_ODOMETRY_NODE_H
