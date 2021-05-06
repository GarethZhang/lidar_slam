//
// Created by haowei on 2021-05-01.
//

#ifndef NEAREST_NEIGHBOR_H
#define NEAREST_NEIGHBOR_H

#include "lidar_slam/archive/nanoflann.hpp"
#include "lidar_slam/archive/common.h"

typedef nanoflann::KDTreeEigenMatrixAdaptor<PMat> kd_tree;

class nearestNeighbor {
public:
    /// Constructor for NN
    /// \param points_in_ref points in reference frame
    /// \param max_leaf maximum leaves allowed
    /// \param num_results number of results returned for KNN, i.e K
    nearestNeighbor(PMat points_in_ref, const size_t max_leaf = 10, const size_t num_results = 3);

    void searchNN(PMat points_in_data, Eigen::MatrixXi& indices, Eigen::MatrixXf& dists);

private:
    PMat points_in_ref_;
    kd_tree index_;
    const size_t num_results_;
};


#endif //NEAREST_NEIGHBOR_H
