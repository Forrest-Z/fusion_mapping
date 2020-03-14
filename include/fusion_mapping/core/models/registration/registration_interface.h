//
// Created by linsin on 17/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_MODELS_REGISTRACTION_REGISTRACTION_INTERFACE_H_
#define FUSION_MAPPING_INCLUDE_MODELS_REGISTRACTION_REGISTRACTION_INTERFACE_H_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "fusion_mapping/core/sensor_data/cloud_data.h"

namespace FM {
class RegistrationInterface {
 public:
  virtual ~RegistrationInterface() = default;

  virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
  virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                         const Eigen::Matrix4f& predict_pose,
                         CloudData::CLOUD_PTR& result_cloud_ptr,
                         Eigen::Matrix4f& result_pose) = 0;
  virtual float GetFitnessScore() = 0;
};
}

#endif //FUSION_MAPPING_INCLUDE_MODELS_REGISTRACTION_REGISTRACTION_INTERFACE_H_
