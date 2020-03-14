//
// Created by linsin on 14/03/2020.
//

#include "fusion_mapping/core/models/cloud_filter/no_filter.h"

namespace FM {
NoFilter::NoFilter() {
}

bool NoFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
  filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
  return true;
}
}