//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_POSE_DATA_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_POSE_DATA_H_
#include <Eigen/Dense>

namespace FM {
class PoseData {
 public:
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double time = 0.0;

 public:
  Eigen::Quaternionf GetQuaternion();
};
}

#
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_POSE_DATA_H_
