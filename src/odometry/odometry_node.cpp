//
// Created by haowei on 2021-05-04.
//

#include "lidar_slam/odometry/odometry_node.h"

OdometryNode::OdometryNode(ros::NodeHandle *nh):
        nh_(*nh){
    getParams();

    cloud_sub_ptr_ = std::make_shared<PointCloudSubscriber>(nh_, velodyne_topic_, queue_size_);

    odometry_ptr_ = std::make_shared<Odometry>(yaml_config_fname_);
}

/// get global parameters for odometry setup
void OdometryNode::getParams() {
    nh_.param<std::string>("velodyne_topic",    velodyne_topic_,     "/velodyne_points");
    nh_.param<float>("queue_size",    queue_size_,     100000);
    nh_.param<std::string>("yaml_config_fname",    yaml_config_fname_,     "");
}

/// read in most recent data from point cloud subscriber buffer and add to current point cloud buffer
/// \return
bool OdometryNode::readData() {
    return false;
}

/// check if current point cloud buffer is empty
/// \return
bool OdometryNode::isDataEmpty() {
    return false;
}

/// get the most recent point cloud and pop it from the buffer front
/// \return
bool OdometryNode::isValidData() {
    return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;
    OdometryNode odometry_node(&nh);
    ros::spin();
    return 0;
}