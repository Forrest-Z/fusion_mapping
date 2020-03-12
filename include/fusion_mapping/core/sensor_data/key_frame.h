//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_KEY_FRAME_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_KEY_FRAME_H_
#include <Eigen/Dense>

namespace FM {
class KeyFrame {
 public:
  double time = 0.0;
  unsigned int index = 0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

 public:
  Eigen::Quaternionf GetQuaternion();
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_KEY_FRAME_H_
