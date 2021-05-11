//
// Created by haowei on 2021-05-10.
//

#include "lidar_slam/publisher/trajectory_publisher.h"

TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle& nh,
                                         std::string& topic,
                                         std::string& frame,
                                         size_t max_path_length,
                                         size_t queue_size) :
        nh_(nh),
        frame_(frame),
        max_path_length_(max_path_length){
    pub_ = nh_.advertise<nav_msgs::Path>(topic, queue_size);
}

void TrajectoryPublisher::publishTrajectory(const Common::TMat& T, ros::Time time) {
    geometry_msgs::Pose pose = PointMatcher_ros::eigenMatrixToPoseMsg<float>(T);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = time;
    pose_stamped.header.frame_id = frame_;
    pose_stamped.pose = pose;

    traj_.header.stamp = time;
    traj_.header.frame_id = frame_;
    if (traj_.poses.size() > max_path_length_) traj_.poses.erase(traj_.poses.begin());
    traj_.poses.push_back(pose_stamped);
    pub_.publish(traj_);
}