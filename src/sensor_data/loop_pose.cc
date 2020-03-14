//
// Created by linsin on 14/03/2020.
//

#include "fusion_mapping/core/sensor_data/loop_pose.h"

namespace FM {
Eigen::Quaternionf LoopPose::GetQuaternion() {
  Eigen::Quaternionf q;
  q = pose.block<3,3>(0,0);

  return q;
}
}