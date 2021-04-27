// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"
// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
//
// Created by haowei on 2021-04-23.
//

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
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "lidar_slam/common.h"
#include "pointmatcher_ros/transform.h"

// define a class, including a constructor, member variables and member functions
class lidar_odometry_class {
public:
    lidar_odometry_class(
            ros::NodeHandle *nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, and publisher
    ros::Subscriber sub_; //these will be set up within the class constructor, hiding these ugly details
    ros::Publisher pub_;

    tf::TransformBroadcaster tf_broadcaster_;

    std::string velodyne_topic_;
    std::string icp_config_fname_;

    // whether this is the first scan
    bool first_scan_;

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

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();

    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &message_holder); //prototype for callback of example subscriber

    void getParams();

    void initialize_icp();

    static void pclPointCloudToEigen(const pcl::PointCloud<pclPointType>& cloudIn, PMat& cloudOut);

    void publishPoseToOdom(PM::TransformationParameters T, nav_msgs::Odometry &msg);

    void sendTransform(const TMat T, std::string src_frame, std::string tgt_frame, ros::Time time);
}; // note: a class definition requires a semicolon at the end of the definition
#endif  // this closes the header-include trick...ALWAYS need one of these to match
