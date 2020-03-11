
#ifndef FUSSION_MAPPING_INCLUDE_UTIL_UTIL_HPP_
#define FUSSION_MAPPING_INCLUDE_UTIL_UTIL_HPP_

#include <cmath>
#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pwd.h>
#include <dirent.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <velodyne_pointcloud/point_types.h>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "mkdir_p.hpp"

#define    COLOR_NONE                    "\033[0m"
#define     FONT_COLOR_RED             "\033[1;31m"
#define    FONT_COLOR_BLUE            "\033[1;34m"
#define    FONT_COLOR_GREEN            "\033[1;32m"


#define DEBUG(fmt, args...) fprintf(stderr, "\033[1;32m  DEBUG(%s:%s:%d):\t\033[0m \n" fmt,__FILE__ , __func__, __LINE__, ## args)

namespace FM {
inline int SetGoogleLogDir() {
  const char *homedir;
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }
  std::string log_path = std::string(homedir) + "/.ros/log/fusion_mapping/";
  struct stat info;
  if (stat(log_path.c_str(), &info) != 0) {
    if (mkdir_p(log_path.c_str()) != 0)
      return -1;
  }
  FLAGS_log_dir = log_path;
  return 0;
}
}

#endif //FUSSION_MAPPING_INCLUDE_UTIL_UTIL_HPP_