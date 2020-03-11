//
// Created by linsin on 17/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_MODELS_REGISTRATION_NDT_REGISTRATION_H_
#define FUSION_MAPPING_INCLUDE_MODELS_REGISTRATION_NDT_REGISTRATION_H_

#include <pcl/registration/ndt.h>
#include "fusion_mapping/core/models/registration/registration_interface.h"

namespace FM {
class NDTRegistration: public RegistrationInterface {
 public:
  NDTRegistration(const YAML::Node& node);
  NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

  bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
  bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                 const Eigen::Matrix4f& predict_pose,
                 CloudData::CLOUD_PTR& result_cloud_ptr,
                 Eigen::Matrix4f& result_pose) override;

 private:
  bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

 private:
  pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
};
}


#endif //FUSION_MAPPING_INCLUDE_MODELS_REGISTRATION_NDT_REGISTRATION_H_
