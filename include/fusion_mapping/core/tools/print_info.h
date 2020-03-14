//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_SRC_TOOLS_PRINT_INFO_H_
#define FUSION_MAPPING_SRC_TOOLS_PRINT_INFO_H_
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"

namespace FM {
class PrintInfo {
 public:
  static void PrintPose(std::string head, Eigen::Matrix4f pose);
};
}
#endif //FUSION_MAPPING_SRC_TOOLS_PRINT_INFO_H_
