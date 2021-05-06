//
// Created by haowei on 2021-05-05.
//

#ifndef LIDAR_SLAM_VOXEL_FILTER_H
#define LIDAR_SLAM_VOXEL_FILTER_H

#include "filter_method.h"
#include <pcl/filters/voxel_grid.h>

class VoxelFilter:public FilterMethod{
public:
    VoxelFilter(const YAML::Node& yaml_config_node);

    void filter(PointCloudData::pointCloudTypePtr& input_cloud, PointCloudData::pointCloudTypePtr& output_cloud);

private:
    pcl::VoxelGrid<PointCloudData::pointType> voxel_filter_;
};


#endif //LIDAR_SLAM_VOXEL_FILTER_H
