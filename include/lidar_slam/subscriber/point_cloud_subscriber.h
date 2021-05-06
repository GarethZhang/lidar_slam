//
// Created by haowei on 2021-05-04.
//

#ifndef LIDAR_SLAM_POINT_CLOUD_SUBSCRIBER_H
#define LIDAR_SLAM_POINT_CLOUD_SUBSCRIBER_H

#include <queue>
#include <mutex>

// ros-related
#include <ros/ros.h>
#include "pcl_conversions/pcl_conversions.h"

// package related
#include "lidar_slam/sensor_data/point_cloud_data.h"

class PointCloudSubscriber{
public:
    PointCloudSubscriber(ros::NodeHandle& nh, std::string topic, size_t queue_size);

    void parseData(std::queue<PointCloudData>& point_cloud_queue);

private:
    void callBack(const sensor_msgs::PointCloud2ConstPtr &msg);

    ros::NodeHandle nh_;

    ros::Subscriber sub_;

    std::queue<PointCloudData> queue_;

    std::mutex mutex_buf_;
};

#endif //LIDAR_SLAM_POINT_CLOUD_SUBSCRIBER_H
