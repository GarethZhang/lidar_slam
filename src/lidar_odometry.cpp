#include <iostream>

#include "pointmatcher/PointMatcher.h"

#include "lidar_slam/utils/utils.h"

int main() {
    std::cout << "Using odometry component in LiDAR SLAM" << std::endl;

    // read in all point cloud filenames under the directory
    std::string data_dir = "/mnt/hdd2/Carla/Town04/03/scans/";
    std::string ext = "ply";
    std::vector<std::string> scan_fnames;

    listdir(data_dir, scan_fnames, ext);

    // read in two consecutive point cloud
    int s = 1000, e = s + 10; // randomly select two frames
    std::string s_scan_fname = scan_fnames[s];
    std::string e_scan_fname = scan_fnames[e];
    Eigen::MatrixXd s_scan, e_scan;
    load_ply(data_dir + s_scan_fname, s_scan);
    load_ply(data_dir + e_scan_fname, e_scan);

    /////////////////////////////////
    // odometry using libpointmatcher
    /////////////////////////////////

    typedef PointMatcher<double> PM;
    typedef PM::DataPoints DP;

    DP::Labels labels;
    labels.push_back(DP::Label("x", 1));
    labels.push_back(DP::Label("y", 1));
    labels.push_back(DP::Label("z", 1));
    labels.push_back(DP::Label("w", 1));

    // load point cloud
    DP ref(s_scan, labels);
    DP data(e_scan, labels);

    // Create the default ICP algorithm
    PM::ICP icp;
    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;

    // Prepare reading filters
    name = "MinDistDataPointsFilter";
    params["minDist"] = "1.0";
    std::shared_ptr<PM::DataPointsFilter> minDist_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

//    name = "RandomSamplingDataPointsFilter";
//    params["prob"] = "0.05";
//    std::shared_ptr<PM::DataPointsFilter> rand_read =
//            PM::get().DataPointsFilterRegistrar.create(name, params);
//    params.clear();

    // Prepare reference filters
    name = "SamplingSurfaceNormalDataPointsFilter";
    params["knn"] = "10";
    params["ratio"] = "0.666666";
    params["samplingMethod"] = "1";
    params["averageExistingDescriptors"] = "0";
    std::shared_ptr<PM::DataPointsFilter> sampSurfNorm_ref =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    // Prepare matching function
    name = "KDTreeMatcher";
    params["knn"] = "1";
    params["epsilon"] = "0";
    std::shared_ptr<PM::Matcher> kdtree =
            PM::get().MatcherRegistrar.create(name, params);
    params.clear();

    // Prepare outlier filters
    name = "TrimmedDistOutlierFilter";
    params["ratio"] = "0.75";
    std::shared_ptr<PM::OutlierFilter> trim =
            PM::get().OutlierFilterRegistrar.create(name, params);
    params.clear();

    // Prepare error minimization
    name = "PointToPlaneErrorMinimizer";
    std::shared_ptr<PM::ErrorMinimizer> pointToPlane =
            PM::get().ErrorMinimizerRegistrar.create(name);

    // Prepare transformation checker filters
    name = "CounterTransformationChecker";
    params["maxIterationCount"] = "40";
    std::shared_ptr<PM::TransformationChecker> maxIter =
            PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    name = "DifferentialTransformationChecker";
    params["minDiffRotErr"] = "0.001";
    params["minDiffTransErr"] = "0.01";
    params["smoothLength"] = "4";
    std::shared_ptr<PM::TransformationChecker> diff =
            PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    // Prepare inspector
    std::shared_ptr<PM::Inspector> nullInspect =
            PM::get().InspectorRegistrar.create("NullInspector");

    // Prepare transformation
    std::shared_ptr<PM::Transformation> rigidTrans =
            PM::get().TransformationRegistrar.create("RigidTransformation");

    // Build ICP solution
    icp.readingDataPointsFilters.push_back(minDist_read);

    icp.referenceDataPointsFilters.push_back(sampSurfNorm_ref);

    icp.matcher = kdtree;

    icp.outlierFilters.push_back(trim);

    icp.errorMinimizer = pointToPlane;

    icp.transformationCheckers.push_back(maxIter);
    icp.transformationCheckers.push_back(diff);

    // toggle to write vtk files per iteration
    icp.inspector = nullInspect;
    //icp.inspector = vtkInspect;

    icp.transformations.push_back(rigidTrans);

    // Compute the transformation to express data in ref
    PM::TransformationParameters T = icp(data, ref);

    // Transform test to express it in ref
    DP data_out(data);
//    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    icp.transformations.apply(data_out, T);

    // Save to disk
    std::string ref_save_fname = "../test/ref.ply";
    std::string data_save_fname = "../test/data.ply";
    std::string data_out_save_fname = "../test/data_out.ply";
    write_ply(ref_save_fname, ref.features);
    write_ply(data_save_fname, data.features);
    write_ply(data_out_save_fname, data_out.features);

    std::cout << "Final transformation:" << std::endl << T << std::endl;

    return 0;
}
