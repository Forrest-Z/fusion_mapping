//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include "fusion_mapping/core/models/scan_adjust/distortion_adjust.h"
#include "fusion_mapping/core/sensor_data/velocity_data.h"
#include "fusion_mapping/core/sensor_data/cloud_data.h"

namespace FM {
class DistortionAdjust {
 public:
  void SetMotionInfo(float scan_period, VelocityData velocity_data);
  bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

 private:
  inline Eigen::Matrix3f UpdateMatrix(float real_time);

 private:
  float scan_period_;
  Eigen::Vector3f velocity_;
  Eigen::Vector3f angular_rate_;
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_
