//
// Created by haowei on 2021-05-09.
//

#include "lidar_slam/tf_broadcaster/tf_broadcaster.h"

TFBroadcaster::TFBroadcaster() {

}

void TFBroadcaster::sendTransform(const Common::TMat& T, const std::string& src_frame, const std::string& tgt_frame, ros::Time time) {
    tf::Transform T_tgt_src = PointMatcher_ros::eigenMatrixToTransform<float>(T);
    tf::StampedTransform T_tgt_src_stamped = tf::StampedTransform(T_tgt_src, time, src_frame, tgt_frame);
    tf_broadcaster_.sendTransform(T_tgt_src_stamped);
}