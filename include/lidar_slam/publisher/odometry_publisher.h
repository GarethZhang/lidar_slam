//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_ODOMETRY_PUBLISHER_H
#define LIDAR_SLAM_ODOMETRY_PUBLISHER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "lidar_slam/common/common.h"

class OdometryPublisher{
public:
    OdometryPublisher(ros::NodeHandle& nh,
                      std::string& topic,
                      std::string& base_frame,
                      std::string& child_frame,
                      size_t queue_size);
    OdometryPublisher() = default;

    void publish(Common::TMat& T_o_s, ros::Time time);

private:
    ros::NodeHandle nh_;

    ros::Publisher pub_;

    nav_msgs::Odometry odometry_;

};

#endif //LIDAR_SLAM_ODOMETRY_PUBLISHER_H
