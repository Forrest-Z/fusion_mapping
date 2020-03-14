//
// Created by linsin on 17/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_MODELS_CLOUD_FILTER_VOXEL_FILTER_H_
#define FUSION_MAPPING_INCLUDE_MODELS_CLOUD_FILTER_VOXEL_FILTER_H_
#include <pcl/filters/voxel_grid.h>
#include "fusion_mapping/core/models/cloud_filter/cloud_filter_interface.h"

namespace FM {
class VoxelFilter: public CloudFilterInterface {
 public:
  VoxelFilter(const YAML::Node& node);
  VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  bool Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr);

 private:
  bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

 private:
  pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
};
}

#endif //FUSION_MAPPING_INCLUDE_MODELS_CLOUD_FILTER_VOXEL_FILTER_H_
