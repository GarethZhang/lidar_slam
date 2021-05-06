//
// Created by haowei on 2021-05-05.
//

#include "lidar_slam/methods/registration/ndt_registration.h"

NDTRegistration::NDTRegistration(const YAML::Node& yaml_config_node) {
    // set ndt params from config
    ndt_.setTransformationEpsilon(yaml_config_node["trans_eps"].as<double>());
    ndt_.setStepSize(yaml_config_node["step_size"].as<double>());
    ndt_.setResolution(yaml_config_node["res"].as<float>());
    ndt_.setMaximumIterations(yaml_config_node["max_iter"].as<int>());
}

void NDTRegistration::setInputTarget(PointCloudData::pointCloudTypePtr& target_cloud) {
    ndt_.setInputTarget(target_cloud);
}

void NDTRegistration::scanRegistration(PointCloudData::pointCloudTypePtr& source_cloud,
                                       PointCloudData::pointCloudTypePtr& output_cloud,
                                       Common::TMat& init_pose,
                                       Common::TMat& pose) {
    ndt_.setInputSource(source_cloud);
    ndt_.align (*output_cloud, init_pose);
    pose = ndt_.getFinalTransformation();
}
