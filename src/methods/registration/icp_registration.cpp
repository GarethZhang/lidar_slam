//
// Created by haowei on 2021-05-05.
//

#include "lidar_slam/methods/registration/icp_registration.h"

ICPRegistration::ICPRegistration(const YAML::Node& yaml_config_node) {
    // set ndt params from config
    icp_.setMaxCorrespondenceDistance (yaml_config_node["max_corr_dist"].as<double>());
    icp_.setMaximumIterations(yaml_config_node["max_iter"].as<int>());
    icp_.setTransformationEpsilon(yaml_config_node["trans_eps"].as<double>());
}

void ICPRegistration::setInputTarget(PointCloudData::pointCloudTypePtr &input_cloud) {
    icp_.setInputTarget(input_cloud);
}

void ICPRegistration::scanRegistration(PointCloudData::pointCloudTypePtr& source_cloud,
                                       PointCloudData::pointCloudTypePtr& output_cloud,
                                       Common::TMat& init_pose,
                                       Common::TMat& pose) {
    icp_.setInputSource(source_cloud);
    icp_.align (*output_cloud, init_pose);
    pose = icp_.getFinalTransformation();
}