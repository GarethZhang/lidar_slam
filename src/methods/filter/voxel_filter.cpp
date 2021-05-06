//
// Created by haowei on 2021-05-05.
//

#include "lidar_slam/methods/filter/voxel_filter.h"

VoxelFilter::VoxelFilter(const YAML::Node& yaml_config_node) {
    // set leaf size
    float leaf_size_x = yaml_config_node["leaf_size"][0].as<float>();
    float leaf_size_y = yaml_config_node["leaf_size"][1].as<float>();
    float leaf_size_z = yaml_config_node["leaf_size"][2].as<float>();
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    LOG(INFO) << "Voxel filter leaf size set to " << leaf_size_x << leaf_size_y << leaf_size_z;
}

void VoxelFilter::filter(PointCloudData::pointCloudTypePtr &input_cloud,
                         PointCloudData::pointCloudTypePtr &output_cloud) {
    voxel_filter_.setInputCloud(input_cloud);
    voxel_filter_.filter(*output_cloud);
}