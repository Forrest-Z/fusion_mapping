//
// Created by linsin on 28/04/2020.
//

#include "fusion_mapping/core/models/ceres_error/parameterization/pose_parameterization.h"

namespace FM{
PoseParameterization::PoseParameterization(const okvis::kinematics::Transformation &T_ws, uint64_t id) {
  setEstimate(T_ws);
}
void PoseParameterization::setEstimate(const okvis::kinematics::Transformation &T_ws) {
  const Eigen::Vector3d r = T_ws.r();
  const Eigen::Vector4d q = T_ws.q().coeffs();
  parameters_[0] = r[0];
  parameters_[1] = r[1];
  parameters_[2] = r[2];
  parameters_[3] = q[0];
  parameters_[4] = q[1];
  parameters_[5] = q[2];
  parameters_[6] = q[3];
}
okvis::kinematics::Transformation PoseParameterization::estimate() const {
  return okvis::kinematics::Transformation(
      Eigen::Vector3d(parameters_[0], parameters_[1], parameters_[2]),
      Eigen::Quaterniond(parameters_[6], parameters_[3], parameters_[4], parameters_[5])
      );
}
}