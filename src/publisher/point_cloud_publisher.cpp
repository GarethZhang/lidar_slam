//
// Created by haowei on 2021-05-08.
//

#include "lidar_slam/publisher/point_cloud_publisher.h"

PointCloudPublisher::PointCloudPublisher(ros::NodeHandle& nh,
                                         std::string& topic,
                                         std::string& frame,
                                         size_t queue_size) :
        nh_(nh),
        frame_(frame){
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, queue_size);
}

void PointCloudPublisher::publish(PointCloudData::pointCloudTypePtr &cloud, ros::Time time) {
    pcl::toROSMsg(*cloud, cloud_);
    cloud_.header.stamp = time;
    cloud_.header.frame_id = frame_;
    pub_.publish(cloud_);
}