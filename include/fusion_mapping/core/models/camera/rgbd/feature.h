//
// Created by linsin on 18/04/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CAMERA_RGBD_FEATURE_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CAMERA_RGBD_FEATURE_H_

#include "fusion_mapping/core/models/camera/rgbd/common_headers.h"

namespace FM {
class Feature {
 public:
  Feature() {}
 public:
  cv::KeyPoint    keypoint;
  cv::Mat         descriptor;
  cv::Point3f     position;       // position in 3D space
  float           observe_frequency = 0.0;      //被观测到的频率
};

}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CAMERA_RGBD_FEATURE_H_
