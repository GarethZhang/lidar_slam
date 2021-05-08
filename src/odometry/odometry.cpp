//
// Created by haowei on 2021-05-04.
//

#include "lidar_slam/odometry/odometry.h"

Odometry::Odometry(std::string& yaml_config_fname) {
    setConfigs(yaml_config_fname);
}

void Odometry::setConfigs(std::string& yaml_config_fname) {
    // load in yaml config
    YAML::Node yaml_config_node = YAML::LoadFile(yaml_config_fname)["Odometry"];

    registration_method_    = yaml_config_node["registration_method"].as<std::string>();
    filter_method_          = yaml_config_node["filter_method"].as<std::string>();
    window_size_            = yaml_config_node["window_size"].as<size_t>();
    submap_create_dist_     = yaml_config_node["submap_create_dist"].as<float>();

    setFilter(yaml_config_node);
    setScanRegistration(yaml_config_node);
}

void Odometry::setFilter(YAML::Node& yaml_config_node) {
    if (filter_method_ == "Voxel"){
        filter_ptr_ = std::make_shared<VoxelFilter> (yaml_config_node[filter_method_]);
        LOG(INFO) << "Filter method" << filter_method_ << "found";
    }
    else{
        LOG(ERROR) << "Filter method" << filter_method_ << "not found";
    }
}

void Odometry::setScanRegistration(YAML::Node& yaml_config_node) {
    if (registration_method_ == "NDT"){
        registration_ptr_ = std::make_shared<NDTRegistration>(yaml_config_node[registration_method_]);
        LOG(INFO) << "Scan registration method" << registration_method_ << "found";
    }
    else if (registration_method_ == "ICP"){
        registration_ptr_ = std::make_shared<ICPRegistration> (yaml_config_node[registration_method_]);
        LOG(INFO) << "Scan registration method" << registration_method_ << "found";
    }
    else{
        LOG(ERROR) << "Scan registration method" << registration_method_ << "not found";
    }
}

void Odometry::updateOdometry(PointCloudData& cloud,
                              Common::TMat& T_o_s_odom) {
    // predict T_o_s using T_sm1_s
    predictSm1S();

    // read in cloud
    curr_frame_.cloud = cloud;

    // if submaps are empty, it means it's the first scan
    // if not do scan to map registration
    if (submaps_.empty()){
        // estimate is identity matrix
        curr_frame_.T_o_s = T_o_s_pred_;

        // add first frame to first submap
        updateSubmap(curr_frame_);

        // for first measurement, set T_sm1_s_ to be identity
        T_sm1_s_ = Common::TMat::Identity();
    }
    else{
        // align the point cloud to submap
        PointCloudData::pointCloudTypePtr aligned_cloud(new PointCloudData::pointCloudType());
        registration_ptr_->scanRegistration(cloud.cloud_ptr, aligned_cloud, T_o_s_pred_, curr_frame_.T_o_s);

        // update incremental pose change
        T_sm1_s_ = T_o_sm1_.inverse() * curr_frame_.T_o_s;
    }

    // add to submap for every X distance travelled
    if (getTravelDist(T_o_lsm_, curr_frame_.T_o_s) > submap_create_dist_){
        updateSubmap(curr_frame_);
    }


    // update last pose and current pose
    T_o_sm1_ = curr_frame_.T_o_s;
    T_o_s_odom = curr_frame_.T_o_s;
}

void Odometry::predictSm1S() {
    if (first_scan_) T_o_s_pred_ = Common::TMat::Identity();
    else T_o_s_pred_ = T_o_sm1_ * T_sm1_s_;
}

void Odometry::updateSubmap(Frame& cloud) {
    submaps_.push_back(cloud);
    if (submaps_.size() > window_size_){
        submaps_.pop_front();
    }

    curr_cloud_->clear();
    PointCloudData::pointCloudTypePtr aligned_submap(new PointCloudData::pointCloudType());
    for (auto & submap : submaps_){
        pcl::transformPointCloud(*submap.cloud.cloud_ptr,
                                 *aligned_submap,
                                 submap.T_o_s);
        *curr_cloud_ += *aligned_submap;
    }

    registration_ptr_->setInputTarget(curr_cloud_);

    T_o_lsm_ = cloud.T_o_s;
}

double Odometry::getTravelDist(Common::TMat& T_o_sm1, Common::TMat& T_o_s) {
    return 0;
}
