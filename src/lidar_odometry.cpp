//
// Created by haowei on 2021-04-22.
//

#include "lidar_slam/lidar_odometry.h"

lidar_odometry_class::lidar_odometry_class(ros::NodeHandle *nodehandle) :
    nh_(*nodehandle),
    first_scan_(true),
    lastPCLCloud_(new pcl::PointCloud<pclPointType>()),
    currPCLCloud_(new pcl::PointCloud<pclPointType>()){ // constructor
    ROS_INFO("in class constructor of lidar_odometry_class");
    getParams();

    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();

    initialize_icp();

    // use default labels for point cloud here
    PMlabels_.push_back(DP::Label("x", 1));
    PMlabels_.push_back(DP::Label("y", 1));
    PMlabels_.push_back(DP::Label("z", 1));
    PMlabels_.push_back(DP::Label("w", 1));

    // set identity for initial odometry
    T_o_s_odom_.setIdentity();
    T_o_s_.setIdentity();
}

//member helper function to set up subscribers;
// note odd syntax: &lidar_odometry_class::subscriberCallback is a pointer to a member function of lidar_odometry_class
// "this" keyword is required, to refer to the current instance of lidar_odometry_class
void lidar_odometry_class::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    sub_ = nh_.subscribe(velodyne_topic_, 1, &lidar_odometry_class::velodyneCallback, this);
    // add more subscribers here, as needed
}

//member helper function to set up publishers;
void lidar_odometry_class::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    pub_ = nh_.advertise<nav_msgs::Odometry>("/lidar_slam/odometry", 1, true);
    //add more publishers, as needed
    // note: COULD make pub_ a public member function, if want to use it within "main()"
}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void lidar_odometry_class::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (first_scan_){
        pcl::fromROSMsg(*msg, *lastPCLCloud_);
        first_scan_ = false;
        return;
    }
    else{
        pcl::fromROSMsg(*msg, *currPCLCloud_);

        ////////////////////////////////
        // use libpointmatcher to do ICP
        ////////////////////////////////
        // convert to Eigen
        PMat lastEigenCloud, currEigenCloud;
        pclPointCloudToEigen(*lastPCLCloud_, lastEigenCloud);
        pclPointCloudToEigen(*currPCLCloud_, currEigenCloud);

        // load into DP
        DP lastPMCloud(lastEigenCloud, PMlabels_);
        DP currPMCloud(currEigenCloud, PMlabels_);

        // apply ICP and compound to odometry
        T_ = icp(currPMCloud, lastPMCloud);
        if (T_o_s_odom_.size() == 0) T_o_s_odom_ = T_;
        else T_o_s_odom_ = T_o_s_odom_ * T_;

        // publish navsat odom msg
        nav_msgs::Odometry odom_msg;
        publishPoseToOdom(T_o_s_odom_, odom_msg);
        odom_msg.header.seq = msg->header.seq;
        odom_msg.header.frame_id = "/map";
        odom_msg.child_frame_id = "/velodyne_odom";
        odom_msg.header.stamp = msg->header.stamp;
        pub_.publish(odom_msg);

        // broadcast the transform between frames
        sendTransform(T_o_s_odom_, "/velodyne_odom", "map", msg->header.stamp);
//        tf::Transform T_o_s_odom = PointMatcher_ros::eigenMatrixToTransform<double>(T_o_s_odom_);
//        tf::StampedTransform T_o_s_odom_stamped
//        T_o_s_broadcaster_.sendTransform()

        // update last cloud
        *lastPCLCloud_ = *currPCLCloud_;
    }
}

void lidar_odometry_class::publishPoseToOdom(PM::TransformationParameters T, nav_msgs::Odometry& msg){
    RMat R_last_curr = T.topLeftCorner(3, 3);
    Vec t_last_curr = T.block<3,1>(0, 3);

    tf::Matrix3x3 R;
    tf::Vector3 t;
    tf::matrixEigenToTF(R_last_curr, R);
    tf::vectorEigenToTF(t_last_curr, t);
    tf::Transform T_last_curr(R, t);

    tf::Quaternion q;
    R.getRotation(q);

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.pose.pose.position.x = t.x();
    msg.pose.pose.position.y = t.y();
    msg.pose.pose.position.z = t.z();
}

void lidar_odometry_class::getParams() {
    nh_.param<std::string>("velodyne_topic",    velodyne_topic_,     "/velodyne_points");
    nh_.param<std::string>("icp_config_fname",  icp_config_fname_,
                           "/home/haowei/MEGA/Research/src/ros_ws/src/lidar_slam/cfg/icp/defaultPointToPlaneMinDistDataPointsFilter.yaml");
}

void lidar_odometry_class::initialize_icp(){
    // load ICP config
    std::ifstream ifs(icp_config_fname_);
    icp.loadFromYaml(ifs);
}

void lidar_odometry_class::pclPointCloudToEigen(const pcl::PointCloud<pclPointType>& cloudIn, PMat& cloudOut) {
    cloudOut = PMat::Ones(4, cloudIn.points.size());
    int i = 0;
    for (auto point:cloudIn.points){
        cloudOut(0, i) = point.x;
        cloudOut(1, i) = point.y;
        cloudOut(2, i) = point.z;
        cloudOut(3, i) = point.intensity;
        i++;
    }
}

void lidar_odometry_class::sendTransform(const TMat T, std::string src_frame, std::string tgt_frame, ros::Time time) {
    tf::Transform T_tgt_src = PointMatcher_ros::eigenMatrixToTransform<double>(T);
    tf::StampedTransform T_tgt_src_stamped = tf::StampedTransform(T_tgt_src, time, src_frame, tgt_frame);
    tf_broadcaster_.sendTransform(T_tgt_src_stamped);
}

int main(int argc, char **argv) {
    // ROS set-ups:
    ros::init(argc, argv, "lidar_odometry_class"); //node name
    ros::NodeHandle nh("~"); // cfreate a node handle; need to pass this to the class constructor
    ROS_INFO("main: instantiating an object of type lidar_odometry_class");
    lidar_odometry_class lidar_odometry_class(
            &nh);  //instantiate an lidar_odometry_class object and pass in pointer to nodehandle for constructor to use
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}