
#include "utils.h"

static inline bool exist(const std::string& name) {
    struct stat buffer{};
    return stat(name.c_str(), &buffer) != 0;
}

void listdir(const std::string& data_dir, std::vector<std::string> &filenames, const std::string& ext) {
    DIR *dir = opendir(data_dir.c_str());
    struct dirent *dp;

    // keep reading in files under directory
    while ((dp = readdir(dir)) != nullptr) {
        std::string filename = dp->d_name;
        if (exist(filename)) { // check if a file exists
            if (!ext.empty()) { // if use an extension for filtering
                std::vector<std::string> splits;
                boost::split(splits, filename, boost::is_any_of("."));
                if (splits[splits.size() - 1] != ext)
                    continue;
            }
            filenames.emplace_back(filename);
        }
    }
    std::sort(filenames.begin(), filenames.end());
}

void load_ply(std::string filename, Eigen::MatrixXd &pointcloud){
    std::vector<PointXYZ> points;
    std::vector<float> float_scalar;
    std::string float_scalar_name = "intensity";
    std::vector<int> int_scalar;
    std::string int_scalar_name = "label";
    load_cloud(filename, points, float_scalar, float_scalar_name, int_scalar, int_scalar_name);
    pointcloud = Eigen::MatrixXd::Ones(4, points.size());
    for (int i = 0; i < points.size(); ++i) {
        pointcloud(0, i) = points[i].x;
        pointcloud(1, i) = points[i].y;
        pointcloud(2, i) = points[i].z;
    }
}

void write_ply(const std::string& filename, Eigen::MatrixXd pointcloud){
    std::vector<PointXYZ> points;
    PointXYZ point;
    for (int i = 0; i < pointcloud.cols(); ++i) {
        point.x = pointcloud(0, i);
        point.y = pointcloud(1, i);
        point.z = pointcloud(2, i);
        points.push_back(point);
    }
    save_cloud(filename, points);
}

