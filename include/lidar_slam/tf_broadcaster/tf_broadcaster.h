//
// Created by haowei on 2021-05-09.
//

#ifndef LIDAR_SLAM_TF_BROADCASTER_H
#define LIDAR_SLAM_TF_BROADCASTER_H

#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/transform.h"

#include "lidar_slam/common/common.h"

class TFBroadcaster{
public:
    TFBroadcaster();

    void sendTransform(const Common::TMat& T, const std::string& src_frame, const std::string& tgt_frame, ros::Time time);

private:
    tf::TransformBroadcaster tf_broadcaster_;
};

#endif //LIDAR_SLAM_TF_BROADCASTER_H
