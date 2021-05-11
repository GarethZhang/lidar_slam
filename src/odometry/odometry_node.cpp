//
// Created by haowei on 2021-05-04.
//

#include "lidar_slam/odometry/odometry_node.h"

OdometryNode::OdometryNode(ros::NodeHandle *nh):
        nh_(*nh),
        frame_num_(0){
    getParams();

    cloud_sub_ptr_ = std::make_shared<PointCloudSubscriber>(nh_, velodyne_topic_, queue_size_);

    odometry_ptr_ = std::make_shared<Odometry>(nh_, yaml_config_fname_);

    odometry_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, odom_topic_, map_frame_, velodyne_frame_, queue_size_);

    cloud_pub_ptr_ = std::make_shared<PointCloudPublisher>(nh_, velodyne_ndt_input_topic_, velodyne_frame_, queue_size_);

    trajectory_pub_ptr_ = std::make_shared<TrajectoryPublisher>(nh_, trajectory_topic_, map_frame_, max_path_length_, queue_size_);

    tf_broadcaster_ptr_ = std::make_shared<TFBroadcaster>();
}

/// get global parameters for odometry setup
void OdometryNode::getParams() {
    nh_.param<std::string>("map_frame",         map_frame_,             "/map");
    nh_.param<std::string>("velodyne_frame",    velodyne_frame_,        "/velodyne");

    nh_.param<std::string>("velodyne_topic",    velodyne_topic_,        "/velodyne_points");
    nh_.param<std::string>("velodyne_ndt_input_topic",    velodyne_ndt_input_topic_,        "/velodyne_ndt_input_topic");
    nh_.param<std::string>("trajectory_topic",    trajectory_topic_,        "/trajectory");
    nh_.param<std::string>("odom_topic",       odom_topic_,            "/odom");
    nh_.param<float>("queue_size",              queue_size_,            100000);

    nh_.param<int>("max_path_length",    max_path_length_,        200);

    nh_.param<std::string>("yaml_config_fname",    yaml_config_fname_,     "/home/haowei/MEGA/Research/src/ros_ws/src/lidar_slam/cfg/odometry/odometry.yaml");
}

/// read in most recent data from point cloud subscriber buffer and add to current point cloud buffer
/// \return
void OdometryNode::readDataFromSub() {
    cloud_sub_ptr_->parseData(queue_);
}

/// check if current point cloud buffer is empty
/// \return
bool OdometryNode::isDataEmpty() {
    if (queue_.empty()) return true;
    return false;
}

/// get the most recent point cloud and pop it from the buffer front
/// \return
void OdometryNode::readData() {
    curr_cloud_ = queue_.front();
    queue_.pop();
}

void OdometryNode::runOdometry() {
    readDataFromSub();

    while (!isDataEmpty()){
        readData();

        LOG(INFO) << "Process " << frame_num_++ << " frame";
        odometry_ptr_->updateOdometry(curr_cloud_, T_o_s_odom_);

        odometry_pub_ptr_->publish(T_o_s_odom_, ros::Time(curr_cloud_.time));

        cloud_pub_ptr_->publish(curr_cloud_.cloud_ptr, ros::Time(curr_cloud_.time));

        tf_broadcaster_ptr_->sendTransform(T_o_s_odom_, map_frame_, velodyne_frame_, ros::Time(curr_cloud_.time));

        trajectory_pub_ptr_->publishTrajectory(T_o_s_odom_, ros::Time(curr_cloud_.time));
    }
}

void OdometryNode::updateOdometry() {
    odometry_ptr_->updateOdometry(curr_cloud_, T_o_s_odom_);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;
    OdometryNode odometry_node(&nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        odometry_node.runOdometry();

        rate.sleep();
    }
    return 0;
}