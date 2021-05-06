//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_COMMON_H
#define LIDAR_SLAM_COMMON_H

#include "pointmatcher/PointMatcher.h"
#include "Eigen/Dense"

class Common{
public:
    using TMat = Eigen::Matrix4d;
    using PMat = Eigen::MatrixXd;
    using RMat = Eigen::Matrix3d;
    using Vec = Eigen::Vector3d;
};

#endif //LIDAR_SLAM_COMMON_H
