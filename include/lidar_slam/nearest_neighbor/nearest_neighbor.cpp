//
// Created by haowei on 2021-05-01.
//

#include "nearest_neighbor.h"

nearestNeighbor::nearestNeighbor(PMat points_in_ref,
                                 const size_t max_leaf,
                                 const size_t num_results) :
        points_in_ref_(points_in_ref),
        index_(3, std::cref(points_in_ref_), max_leaf),
        num_results_(num_results){
    index_.index->buildIndex();
}

void nearestNeighbor::searchNN(PMat points_in_data, Eigen::MatrixXi& indices, Eigen::MatrixXf& dists) {
    indices.resize(points_in_data.rows(), num_results_);
    dists.resize(points_in_data.rows(), num_results_);
    for (int i = 0; i < points_in_data.rows(); ++i) {
        std::vector<double> query_pt{points_in_data(i, 0), points_in_data(i, 1), points_in_data(i, 2)};

        std::vector<size_t> ret_indices(num_results_);
        std::vector<double> out_dists_sqr(num_results_);
        nanoflann::KNNResultSet<double> resultSet(num_results_);
        resultSet.init(&ret_indices[0], &out_dists_sqr[0]);
        index_.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
        for (size_t j = 0; j < num_results_; ++j) {
            indices(i, j) = ret_indices[j];
            dists(i, j) = std::sqrt(out_dists_sqr[j]);
        }
    }
}
