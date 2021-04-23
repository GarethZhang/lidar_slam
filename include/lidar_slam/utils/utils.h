//
// Created by haowei on 2021-04-19.
//

#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>

#include "dirent.h"
#include "sys/stat.h"
#include "boost/algorithm/string.hpp"
#include "Eigen/Dense"

#include "semantic_labeller/cloud/cloud.h"

/// List out all files under a directory
/// \param data_dir directory string
/// \param filenames name of all files
/// \param ext extension to filter out files
void listdir(const std::string& data_dir, std::vector<std::string> &filenames, const std::string& ext);

/// Check if a file exists
/// \param name file name
/// \return whether file exists
bool exist(const char *name);

void load_ply(std::string filename, Eigen::MatrixXd &pointcloud);

void write_ply(const std::string& filename, Eigen::MatrixXd pointcloud);




#endif //UTILS_H
