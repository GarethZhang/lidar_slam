//
// Created by haowei on 2021-04-30.
//

#ifndef SE3_UTILS_H
#define SE3_UTILS_H

#include "../common.h"

template <typename T>
Eigen::Matrix<T,3,3> get_roll(T roll){
    Eigen::Matrix<T,3,3> R = Eigen::Matrix<T,3,3>::Identity();
    R(0, 0) = 1.0;
    R(1, 1) = cos(roll);
    R(1, 2) = sin(roll);
    R(2, 1) = -sin(roll);
    R(2, 2) = cos(roll);
    return R;
}

template <typename T>
Eigen::Matrix<T,3,3> get_pitch(T pitch){
    Eigen::Matrix<T,3,3> R = Eigen::Matrix<T,3,3>::Identity();
    R(0, 0) = cos(pitch);
    R(0, 2) = -sin(pitch);
    R(1, 1) = 1.0;
    R(2, 0) = sin(pitch);
    R(2, 2) = cos(pitch);
    return R;
}

template <typename T>
Eigen::Matrix<T,3,3> get_yaw(T yaw){
    Eigen::Matrix<T,3,3> R = Eigen::Matrix<T,3,3>::Identity();
    R(2, 2) = 1.0;
    R(0, 0) = cos(yaw);
    R(0, 1) = sin(yaw);
    R(1, 0) = -sin(yaw);
    R(1, 1) = cos(yaw);
    return R;
}

template <typename T>
void from_RPY(Eigen::Matrix<T,3,3>& R, T roll, T pitch, T yaw){
    R.setIdentity();
    R = get_yaw<T>(yaw) * get_pitch<T>(pitch) * get_roll<T>(roll);
}

#endif //SE3_UTILS_H
