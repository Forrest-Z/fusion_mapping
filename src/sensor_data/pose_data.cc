//
// Created by linsin on 12/03/2020.
//

#include "fusion_mapping/core/sensor_data/pose_data.h"

namespace FM {
Eigen::Quaternionf PoseData::GetQuaternion() {
  Eigen::Quaternionf q;
  q = pose.block<3,3>(0,0);

  return q;
}
}