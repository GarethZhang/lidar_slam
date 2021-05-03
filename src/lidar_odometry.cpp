
#include "lidar_slam/lidar_odometry.h"

lidar_odometry_class::lidar_odometry_class(ros::NodeHandle *nodehandle) :
    nh_(*nodehandle),
    lastPCLCloud_(new pcl::PointCloud<pclPointType>()),
    currPCLCloud_(new pcl::PointCloud<pclPointType>()){ // constructor
    ROS_INFO("in class constructor of lidar_odometry_class");
    getParams();

    initializeVariables();
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();

    initializeICP();

    // use default labels for point cloud here
    PMlabels_.push_back(DP::Label("x", 1));
    PMlabels_.push_back(DP::Label("y", 1));
    PMlabels_.push_back(DP::Label("z", 1));
    PMlabels_.push_back(DP::Label("w", 1));

    // set identity for initial odometry
    T_o_s_odom_.setIdentity();
    T_o_s_.setIdentity();
}

void lidar_odometry_class::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
    sub_ = nh_.subscribe(velodyne_topic_, 1, &lidar_odometry_class::velodyneCallback, this);
}

void lidar_odometry_class::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    pub_ = nh_.advertise<nav_msgs::Odometry>("/lidar_slam/odometry", 1, true);
    traj_pub_ = nh_.advertise<nav_msgs::Path>("/lidar_slam/odom_traj", 1, true);
}

void lidar_odometry_class::initializeVariables() {
    seq_num_ = -1;
    num_skipped_scans_ = 0;
    first_scan_ = true;
    max_path_length_ = 200;
    dist_travel_ = 0.0;
}

void lidar_odometry_class::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    checkSeqNum(msg->header.seq);

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

        DP data_out(currPMCloud);
        icp.transformations.apply(data_out, T_);

        // update distance travelled
        updateDistTravel(T_);

        // broadcast the transform between frames
        sendTransform(T_o_s_odom_, "/map", "/velodyne", msg->header.stamp);

        // publish path
        publishPath(T_o_s_odom_, "/map", msg->header.stamp);

        // update last cloud
        *lastPCLCloud_ = *currPCLCloud_;
    }
}

void lidar_odometry_class::getParams() {
    nh_.param<std::string>("velodyne_topic",    velodyne_topic_,     "/velodyne_points");
    nh_.param<std::string>("icp_config_fname",  icp_config_fname_,
                           "/home/haowei/MEGA/Research/src/ros_ws/src/lidar_slam/cfg/icp/defaultPointToPlaneMinDistDataPointsFilter.yaml");
}

void lidar_odometry_class::initializeICP(){
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

void lidar_odometry_class::sendTransform(const TMat& T, const std::string& src_frame, const std::string& tgt_frame, ros::Time time) {
    tf::Transform T_tgt_src = PointMatcher_ros::eigenMatrixToTransform<double>(T);
    tf::StampedTransform T_tgt_src_stamped = tf::StampedTransform(T_tgt_src, time, src_frame, tgt_frame);
    tf_broadcaster_.sendTransform(T_tgt_src_stamped);
}

void lidar_odometry_class::publishPath(const TMat& T, const std::string& frame, ros::Time time) {
    geometry_msgs::Pose pose = PointMatcher_ros::eigenMatrixToPoseMsg<double>(T);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = time;
    pose_stamped.header.frame_id = frame;
    pose_stamped.pose = pose;

    traj_path_.header.stamp = time;
    traj_path_.header.frame_id = frame;
    if (traj_path_.poses.size() > max_path_length_) traj_path_.poses.erase(traj_path_.poses.begin());
    traj_path_.poses.push_back(pose_stamped);
    traj_pub_.publish(traj_path_);
}

void lidar_odometry_class::checkSeqNum(const uint32_t& seq_num) {
    auto skipped_scans = seq_num - seq_num_ - 1;
    if (skipped_scans > 0){
        num_skipped_scans_ += skipped_scans;
        ROS_INFO("Skip %d (%d) scans", skipped_scans, num_skipped_scans_);
    }
    seq_num_ = seq_num;
}

void lidar_odometry_class::saveForVis(const TMat &T, const std::string &fname) {
    write_ply(fname, T);
}

void lidar_odometry_class::updateDistTravel(const TMat &T) {
    dist_travel_ += sqrt(T(0, 3) * T(0, 3) +
                            T(1, 3) * T(1, 3) +
                            T(2, 3) * T(2, 3));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_odometry_class"); //node name
    ros::NodeHandle nh("~"); // cfreate a node handle; need to pass this to the class constructor
    ROS_INFO("main: instantiating an object of type lidar_odometry_class");
    lidar_odometry_class lidar_odometry_class(
            &nh);  //instantiate an lidar_odometry_class object and pass in pointer to nodehandle for constructor to use
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}