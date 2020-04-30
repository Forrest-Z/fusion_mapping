//
// Created by linsin on 30/04/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CERES_ERROR_LOCAL_PARAM_POSE_LOCAL_PARAMETERIZATION_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CERES_ERROR_LOCAL_PARAM_POSE_LOCAL_PARAMETERIZATION_H_

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "okvis/kinematics/Transformation.hpp"

namespace FM{
class PoseLocalParameterization: public ceres::LocalParameterization {
  virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double *jacobian) const;
  virtual int GlobalSize() const {return 7;}
  virtual int LocalSize() const {return 6;}
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CERES_ERROR_LOCAL_PARAM_POSE_LOCAL_PARAMETERIZATION_H_
