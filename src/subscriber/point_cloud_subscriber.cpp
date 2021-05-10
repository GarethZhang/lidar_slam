//
// Created by haowei on 2021-05-04.
//

#include "lidar_slam/subscriber/point_cloud_subscriber.h"

PointCloudSubscriber::PointCloudSubscriber(ros::NodeHandle& nh, std::string topic, size_t queue_size):
        nh_(nh){
    sub_ = nh_.subscribe(topic, queue_size, &PointCloudSubscriber::callBack, this);
}

void PointCloudSubscriber::callBack(const sensor_msgs::PointCloud2ConstPtr &msg) {
    mutex_buf_.lock();
    PointCloudData point_cloud;
    pcl::fromROSMsg(*msg, *point_cloud.cloud_ptr);
    point_cloud.time = msg->header.stamp.toSec();
    queue_.push(point_cloud);
    mutex_buf_.unlock();
}

void PointCloudSubscriber::parseData(std::queue<PointCloudData> &point_cloud_queue) {
    mutex_buf_.lock();
    if (!queue_.empty()){
        point_cloud_queue.push(queue_.front());
        queue_.pop();
    }
    mutex_buf_.unlock();
}
