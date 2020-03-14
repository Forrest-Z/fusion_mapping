//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_LOOP_POSE_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_LOOP_POSE_H_
#include <Eigen/Dense>

namespace FM {
class LoopPose {
 public:
  double time = 0.0;
  unsigned int index0 = 0;
  unsigned int index1 = 0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

 public:
  Eigen::Quaternionf GetQuaternion();
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_LOOP_POSE_H_
