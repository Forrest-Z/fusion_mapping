//
// Created by linsin on 28/04/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CERES_ERROR_CALIBRATION_PARAMETERIZATION_POSE_PARAMETERIZATION_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CERES_ERROR_CALIBRATION_PARAMETERIZATION_POSE_PARAMETERIZATION_H_

#include <Eigen/Dense>
#include "fusion_mapping/core/okvis/kinematics/Transformation.hpp"

namespace FM{
class PoseParameterization{
 public:
  typedef okvis::kinematics::Transformation estimate_t;
  PoseParameterization() = default;
  PoseParameterization(const okvis::kinematics::Transformation& T_ws, uint64_t id);
  ~PoseParameterization(){};
  void setEstimate(const okvis::kinematics::Transformation& T_ws); // read from init guss
  okvis::kinematics::Transformation estimate() const; //return estimated value after optimization
  double* parameters() {
    return parameters_;
  } int dimension() const {
    return 7;
  }
 private:
  double parameters_[7] = {0};
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CERES_ERROR_CALIBRATION_PARAMETERIZATION_POSE_PARAMETERIZATION_H_
