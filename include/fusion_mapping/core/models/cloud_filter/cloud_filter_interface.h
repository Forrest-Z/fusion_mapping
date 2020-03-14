//
// Created by linsin on 17/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define FUSION_MAPPING_INCLUDE_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <iostream>
#include "fusion_mapping/core/sensor_data/cloud_data.h"

namespace FM {
class CloudFilterInterface {
 public:
  virtual ~CloudFilterInterface() = default;

  virtual bool Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}


#endif //FUSION_MAPPING_INCLUDE_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
