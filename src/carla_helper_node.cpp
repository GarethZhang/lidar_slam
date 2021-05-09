// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
//#include <opencv2/opencv.hpp>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <nav_msgs/Odometry.h>
//#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "geometry_msgs/PoseStamped.h"

#include "lidar_slam/utils/utils.h"
#include "semantic_labeller/cloud/cloud.h"
#include "velodyne_pointcloud/rawdata.h"

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "carla_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder, town_number, sequence_number, velodyne_topic, pose_topic, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("town_number", town_number);
    n.getParam("sequence_number", sequence_number);
    n.getParam("velodyne_topic", velodyne_topic);
    n.getParam("pose_topic", pose_topic);
    std::cout << "Reading sequence " << sequence_number << " from " << town_number << " from " << dataset_folder << '\n';
    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>(velodyne_topic, 1, true);
    ros::Publisher pub_laser_pose = n.advertise<geometry_msgs::PoseStamped>(pose_topic, 1, true);

//    image_transport::ImageTransport it(n);
//    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
//    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);
//
//    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
//    nav_msgs::Odometry odomGT;
//    odomGT.header.frame_id = "/camera_init";
//    odomGT.child_frame_id = "/ground_truth";

//    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + town_number + sequence_number + "timestamps.txt", std::ifstream::in);
    std::ifstream transforms_file(dataset_folder + town_number + sequence_number + "transforms.txt", std::ifstream::in);

//    std::string ground_truth_path = "results/" + sequence_number + ".txt";
//    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);

    rosbag::Bag bag_out;
    if (to_bag){
        output_bag_file = dataset_folder + town_number + sequence_number + output_bag_file;
        bag_out.open(output_bag_file, rosbag::bagmode::Write);
    }

//    Eigen::Matrix3d R_transform;
//    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
//    Eigen::Quaterniond q_transform(R_transform);

    // Read all LiDAR filenames
    std::string scan_dir = dataset_folder + town_number + sequence_number + "scans/";
    std::string ext = "ply";
    std::vector<std::string> scan_fnames;
    listdir(scan_dir, scan_fnames, ext);

    std::string timestamp_line, transforms_line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay);
    while (std::getline(timestamp_file, timestamp_line) && std::getline(transforms_file, transforms_line) && ros::ok())
    {
        float timestamp = stof(timestamp_line);

        std::string lidar_data_path = scan_dir + scan_fnames[line_num];
        ROS_INFO("lidar_data_path: %s", lidar_data_path.c_str());
        std::vector<PointXYZ> lidar_data;
        std::vector<float> float_scalar;
        std::string float_scalar_name = "intensity";
        std::vector<int> int_scalar;
        std::string int_scalar_name = "label";
        load_cloud(lidar_data_path, lidar_data, float_scalar, float_scalar_name, int_scalar, int_scalar_name);
        std::cout << "totally " << lidar_data.size() << " points in this lidar frame \n";

        velodyne_rawdata::VPointCloud::Ptr laser_cloud_msg(new velodyne_rawdata::VPointCloud());
        velodyne_rawdata::VPoint point;

        for (std::size_t i = 0; i < lidar_data.size(); i++){
            point.x = lidar_data[i].x;
            point.y = lidar_data[i].y;
            point.z = lidar_data[i].z;
            point.intensity = float_scalar[i];
            point.ring = int_scalar[i];
            point.timeINS = 0.0;
            laser_cloud_msg->points.push_back(point);
            laser_cloud_msg->width++;
        }
        laser_cloud_msg->header.stamp = (uint64_t) (timestamp * 1e6);
        laser_cloud_msg->header.frame_id = "/velodyne";
        laser_cloud_msg->height = 1;

        pub_laser_cloud.publish(laser_cloud_msg);

        // get ground truth transform
        std::vector<std::string> words;
        boost::split(words, transforms_line, boost::is_any_of(" "), boost::token_compress_on);
        Eigen::Matrix4d T_o_s;
        T_o_s(0, 0) = atof(words[0].c_str());
        T_o_s(0, 1) = atof(words[1].c_str());
        T_o_s(0, 2) = atof(words[2].c_str());
        T_o_s(0, 3) = atof(words[3].c_str());
        T_o_s(1, 0) = atof(words[4].c_str());
        T_o_s(1, 1) = atof(words[5].c_str());
        T_o_s(1, 2) = atof(words[6].c_str());
        T_o_s(1, 3) = atof(words[7].c_str());
        T_o_s(2, 0) = atof(words[8].c_str());
        T_o_s(2, 1) = atof(words[9].c_str());
        T_o_s(2, 2) = atof(words[10].c_str());
        T_o_s(2, 3) = atof(words[11].c_str());
        T_o_s(3, 0) = atof(words[12].c_str());
        T_o_s(3, 1) = atof(words[13].c_str());
        T_o_s(3, 2) = atof(words[14].c_str());
        T_o_s(3, 3) = atof(words[15].c_str());
        Eigen::Matrix3d R_o_s = T_o_s.block<3,3>(0,0);
        Eigen::Vector3d t_o_s = T_o_s.block<3,1>(0,3);
        Eigen::Quaterniond q(R_o_s);

        geometry_msgs::PoseStamped pose_stamped;
        geometry_msgs::Pose pose;
        pose.position.x = t_o_s(0);
        pose.position.y = t_o_s(1);
        pose.position.z = t_o_s(2);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose_stamped.pose = pose;

        uint32_t sec = (long) timestamp;
        uint32_t nsec = (long) ((timestamp - sec) * 1e9);
        pose_stamped.header.stamp = ros::Time(sec, nsec);
        pose_stamped.header.frame_id = "/map";
        pub_laser_pose.publish(pose_stamped);

        if (to_bag)
        {
            bag_out.write(velodyne_topic, ros::Time::now(), laser_cloud_msg);
            bag_out.write(pose_topic, ros::Time::now(), pose_stamped);
        }

        line_num ++;
        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";


    return 0;
}