//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CLOUD_FILTER_BOX_FILTER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CLOUD_FILTER_BOX_FILTER_H_
#include <pcl/filters/crop_box.h>
#include "fusion_mapping/core/models/cloud_filter/cloud_filter_interface.h"

namespace FM {
class BoxFilter: public CloudFilterInterface {
 public:
  BoxFilter(YAML::Node node);
  BoxFilter() = default;

  bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  void SetSize(std::vector<float> size);
  void SetOrigin(std::vector<float> origin);
  std::vector<float> GetEdge();

 private:
  void CalculateEdge();

 private:
  pcl::CropBox<CloudData::POINT> pcl_box_filter_;

  std::vector<float> origin_;
  std::vector<float> size_;
  std::vector<float> edge_;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CLOUD_FILTER_BOX_FILTER_H_
