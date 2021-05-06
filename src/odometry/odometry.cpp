//
// Created by haowei on 2021-05-04.
//

#include "lidar_slam/odometry/odometry.h"

Odometry::Odometry(std::string& yaml_config_fname) {
    setConfigs(yaml_config_fname);
}

void Odometry::setConfigs(std::string& yaml_config_fname) {
    // load in yaml config
    YAML::Node yaml_config_node = YAML::LoadFile(yaml_config_fname);

    registration_method_    = yaml_config_node["registration_method"].as<std::string>();
    filter_method_          = yaml_config_node["filter_method"].as<std::string>();

    setFilter(yaml_config_node);
    setScanRegistration(yaml_config_node);
}

void Odometry::setFilter(YAML::Node& yaml_config_node) {
    if (filter_method_ == "Voxel"){
        filter_ptr_ = std::make_shared<VoxelFilter> ();
    }
    else{
        LOG(ERROR) << "Filter method" << filter_method_ << "not found";
    }
}

void Odometry::setScanRegistration(YAML::Node& yaml_config_node) {
    if (registration_method_ == "NDT"){
        registration_ptr_ = std::make_shared<NDTRegistration>(yaml_config_node[registration_method_]);
    }
    else if (registration_method_ == "ICP"){
        registration_ptr_ = std::make_shared<ICPRegistration> (yaml_config_node[registration_method_]);
    }
    else{
        LOG(ERROR) << "Scan registration method" << registration_method_ << "not found";
    }
}
