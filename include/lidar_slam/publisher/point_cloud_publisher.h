//
// Created by haowei on 2021-05-08.
//

#ifndef LIDAR_SLAM_POINT_CLOUD_PUBLISHER_H
#define LIDAR_SLAM_POINT_CLOUD_PUBLISHER_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_slam/sensor_data/point_cloud_data.h"

class PointCloudPublisher{
public:
    PointCloudPublisher(ros::NodeHandle& nh,
                        std::string& topic,
                        std::string& frame,
                        size_t queue_size);
    PointCloudPublisher() = default;

    void publish(PointCloudData::pointCloudTypePtr& cloud,
                 ros::Time time);
private:
    ros::NodeHandle nh_;

    ros::Publisher pub_;

    std::string frame_;

    sensor_msgs::PointCloud2 cloud_;
};

#endif //LIDAR_SLAM_POINT_CLOUD_PUBLISHER_H
