
#ifndef LIDAR_SLAM_LIDAR_ODOMETRY_H
#define LIDAR_SLAM_LIDAR_ODOMETRY_H
//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "utils/utils.h"
#include "lidar_slam/common.h"
#include "pointmatcher_ros/transform.h"

class lidar_odometry_class {
public:
    lidar_odometry_class(
            ros::NodeHandle *nodehandle);
private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_;
    ros::Publisher pub_, traj_pub_;

    tf::TransformBroadcaster tf_broadcaster_;

    nav_msgs::Path traj_path_;
    uint32_t max_path_length_;

    std::string velodyne_topic_;
    std::string icp_config_fname_;

    bool first_scan_; // whether this is the first scan
    uint32_t seq_num_, num_skipped_scans_;
    double dist_travel_;

    // default labels for point cloud
    DP::Labels PMlabels_;

    // icp
    PM::ICP icp;
    PM::TransformationParameters T_o_s_odom_;
    PM::TransformationParameters T_;
    PM::TransformationParameters T_o_s_;

    // pointers to hold point cloud data
    pcl::PointCloud<pclPointType>::Ptr lastPCLCloud_;
    pcl::PointCloud<pclPointType>::Ptr currPCLCloud_;

    // intializers
    void initializeSubscribers();
    void initializePublishers();
    void initializeVariables();
    void initializeICP();
    void getParams();

    // Callbacks
    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &message_holder);

    // Utility Functions
    static void pclPointCloudToEigen(const pcl::PointCloud<pclPointType>& cloudIn, PMat& cloudOut);
    void sendTransform(const TMat& T, const std::string& src_frame, const std::string& tgt_frame, ros::Time time);
    void publishPath(const TMat& T, const std::string& frame, ros::Time time);
    void saveForVis(const TMat& T, const std::string& fname);
    void updateDistTravel(const TMat& T);
    void checkSeqNum(const uint32_t& seq_num);

};
#endif
