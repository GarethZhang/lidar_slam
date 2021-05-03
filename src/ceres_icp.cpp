#include "lidar_slam/nanoflann.hpp"
#include "lidar_slam/common.h"

#include "semantic_labeller/cloud/cloud.h"
#include "lidar_slam/se3_utils/se3_utils.h"
#include "lidar_slam/nearest_neighbor/nearest_neighbor.h"

#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>
#include <ceres/ceres.h>

struct ICPCeres
{
    ICPCeres (Vec point1_in_ref, Vec point2_in_data) :
    point1_in_ref_(point1_in_ref),
    point2_in_data_(point2_in_data) {}

    // calculate residuals
    template <typename T>
    bool operator() (
            const T * const rotation, // quaternion
            const T* const t, // translation vector
            T * residual) const // residuals
    {
        // read in two points
        Eigen::Matrix<T,3,1> points1; points1 << T(point1_in_ref_[0]), T(point1_in_ref_[1]), T(point1_in_ref_[2]);
        Eigen::Matrix<T,3,1> points2; points2 << T(point2_in_data_[0]), T(point2_in_data_[1]), T(point2_in_data_[2]);

        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(rotation);

        Eigen::Matrix<T, 3, 1> transVec;
        transVec << T(t[0]), T(t[1]), T(t[2]);

        Eigen::Matrix<T, 3, 1> point2_in_ref = q * points2 + transVec;

        residual[0] = points1[0] - point2_in_ref[0];
        residual[1] = points1[1] - point2_in_ref[1];
        residual[2] = points1[2] - point2_in_ref[2];
        return true;
    }
    static ceres::CostFunction* Create(const Vec point1_in_ref,const Vec point2_in_data) {
        return (new ceres::AutoDiffCostFunction<ICPCeres, 3, 4, 3>(
                new ICPCeres(point1_in_ref, point2_in_data)));
    }
    const Vec point1_in_ref_;
    const Vec point2_in_data_;
};


int main ( int argc, char** argv ) {
    // load one point cloud
    std::vector<PointXYZ> points;
    std::string fname = "/home/haowei/MEGA/Research/src/ros_ws/src/lidar_slam/test/data.ply";
    load_cloud(fname, points);
    PMat points2_in_data(points.size(), 4);
    for (int i = 0; i < points.size(); i++){
        points2_in_data(i, 0) = points[i].x;
        points2_in_data(i, 1) = points[i].y;
        points2_in_data(i, 2) = points[i].z;
        points2_in_data(i, 3) = 1;
    }

    // apply a random transformation
    double roll = 0.01;
    double pitch = 0.002;
    double yaw = 0.1;
    Eigen::Matrix<double, 3, 3> R_ref_data;
    from_RPY<double>(R_ref_data, roll, pitch, yaw);
    Vec t_data_ref_in_ref(0.3, 0.02, 0.03);
    TMat T_ref_data;
    T_ref_data.block<3,3>(0, 0) = R_ref_data;
    T_ref_data.block<3,1>(0,3) = t_data_ref_in_ref;

    PMat points1_in_ref = (T_ref_data * points2_in_data.transpose()).transpose();

    // initialize reference point cloud for NN search
    // this kd tree is only built once for all
    // update points_in_data for further use
    PMat points1_in_ref_coords = points1_in_ref.leftCols(3);
    nearestNeighbor nn_ref(points1_in_ref_coords);
    Eigen::MatrixXi indices;
    Eigen::MatrixXf dists;

    int iter_i = 0;
    TMat T_ref_data_icp = TMat::Identity(); //initial guess into icp
    std::cout << "Initial: \n" << T_ref_data_icp << std::endl;
    while (iter_i < 40){
        // start optimization using ceres
        ceres::Problem problem;
        ceres::CostFunction* cost_function;

        // always identity as initial guess
        Eigen::Quaterniond q(RMat::Identity());
        Vec t = Vec::Zero();

        // establish closest point correspondence using initial guess
        PMat points2_in_data_coords = (T_ref_data_icp * points2_in_data.transpose()).transpose().leftCols(3);
        nn_ref.searchNN(points2_in_data_coords, indices, dists);

        Vec point1_in_ref;
        Vec point2_in_data;

        // loop through every point and add to problem
        for (int i = 0; i < points2_in_data.rows(); i++){
            point1_in_ref.setZero();
            point1_in_ref << points1_in_ref_coords(indices(i), 0), points1_in_ref_coords(indices(i), 1), points1_in_ref_coords(indices(i), 2);
            point2_in_data.setZero();
            point2_in_data << points2_in_data_coords(i, 0), points2_in_data_coords(i, 1), points2_in_data_coords(i, 2);

            cost_function = ICPCeres::Create(point1_in_ref, point2_in_data);
            ceres::LossFunction* loss = NULL;
            problem.AddResidualBlock(cost_function, loss, q.coeffs().data(), t.data());
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
//        std::cout << summary.FullReport() << "\n";

        // update T_ref_data_icp
        TMat T_last_cur = TMat::Identity();
        T_last_cur.block<3,3>(0,0) = q.toRotationMatrix();
        T_last_cur.block<3,1>(0,3) = t;
        T_ref_data_icp = T_ref_data_icp * T_last_cur;

        iter_i++;
    }
    std::cout << "Groundtruth: \n" << T_ref_data << std::endl;
    std::cout << "Final: \n" << T_ref_data_icp << std::endl;
}