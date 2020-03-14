//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CLOUD_FILTER_NO_FILTER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CLOUD_FILTER_NO_FILTER_H_
#include "fusion_mapping/core/models/cloud_filter/cloud_filter_interface.h"

namespace FM {
class NoFilter: public CloudFilterInterface {
 public:
  NoFilter();

  bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CLOUD_FILTER_NO_FILTER_H_
