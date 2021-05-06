//
// Created by haowei on 2021-04-23.
//

#ifndef COMMON_H
#define COMMON_H

#include "pcl/point_types.h"
#include "pointmatcher/PointMatcher.h"

typedef pcl::PointXYZI pclPointType;

typedef Eigen::Matrix4d TMat;
typedef Eigen::MatrixXd PMat;
typedef Eigen::Matrix3d RMat;
typedef Eigen::Vector3d Vec;

typedef PointMatcher<double> PM;
typedef PM::DataPoints DP;

#endif //COMMON_H
