//
// Created by haowei on 2021-05-05.
//

#include "lidar_slam/publisher/odometry_publisher.h"

OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh,
                                     std::string& topic,
                                     std::string& base_frame,
                                     std::string& child_frame,
                                     size_t queue_size):
        nh_(nh),
        pub_(nh_.advertise<nav_msgs::Odometry>(topic, queue_size)){
    odometry_.header.frame_id = base_frame;
    odometry_.child_frame_id = child_frame;
}

void OdometryPublisher::publish(Common::TMat &T_o_s, ros::Time time) {

    odometry_.header.stamp = time;

    odometry_.pose.pose.position.x = T_o_s(0,3);
    odometry_.pose.pose.position.y = T_o_s(1,3);
    odometry_.pose.pose.position.z = T_o_s(2,3);

    Eigen::Quaternionf q;
    q = T_o_s.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    pub_.publish(odometry_);
}