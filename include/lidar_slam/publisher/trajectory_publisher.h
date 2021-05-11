//
// Created by haowei on 2021-05-10.
//

#ifndef LIDAR_SLAM_TRAJECTORY_PUBLISHER_H
#define LIDAR_SLAM_TRAJECTORY_PUBLISHER_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"

#include "pointmatcher_ros/transform.h"

#include "lidar_slam/common/common.h"

class TrajectoryPublisher{
public:
    TrajectoryPublisher(ros::NodeHandle& nh,
                        std::string& topic,
                        std::string& frame,
                        size_t max_path_length,
                        size_t queue_size);

    void publishTrajectory(const Common::TMat& T, ros::Time time);

private:
    ros::NodeHandle nh_;

    ros::Publisher pub_;

    nav_msgs::Path traj_;

    std::string frame_;

    size_t max_path_length_;

};

#endif //LIDAR_SLAM_TRAJECTORY_PUBLISHER_H
