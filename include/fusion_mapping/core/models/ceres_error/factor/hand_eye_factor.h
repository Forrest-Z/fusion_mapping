//
// Created by linsin on 28/04/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CERES_ERROR_CALIBRATION_TERM_HAND_EYE_FACTOR_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CERES_ERROR_CALIBRATION_TERM_HAND_EYE_FACTOR_H_

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "fusion_mapping/core/okvis/kinematics/Transformation.hpp"
#include "fusion_mapping/core/models/ceres_error/parameterization/pose_parameterization.h"

namespace FM{
 class HandEyeFactor : public ceres::SizedCostFunction<1, 7> {
  public:
   HandEyeFactor() = delete;
   HandEyeFactor(const PoseParameterization& lidar_odom, const PoseParameterization& gnss_odom);
   ~HandEyeFactor(){};
   virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
   void setMeasurementsAndGNSS(const PoseParameterization& lidar_odom, const PoseParameterization& gnss_odom){
     lidar_odom_ = lidar_odom;
     gnss_odom_ = gnss_odom;
   }
  private:
   PoseParameterization lidar_odom_;
   PoseParameterization gnss_odom_;
 };// residual, one pose
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CERES_ERROR_CALIBRATION_TERM_HAND_EYE_FACTOR_H_
