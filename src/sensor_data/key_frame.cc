//
// Created by linsin on 12/03/2020.
//

#include "fusion_mapping/core/sensor_data/key_frame.h"

namespace FM {
Eigen::Quaternionf KeyFrame::GetQuaternion() {
  Eigen::Quaternionf q;
  q = pose.block<3,3>(0,0);

  return q;
}
}