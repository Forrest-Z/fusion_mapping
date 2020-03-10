//
// Created by linsin on 10/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_CLOUD_DATA_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_CLOUD_DATA_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace FM {
class CloudData {
 public:
  using POINT = pcl::PointXYZ;
  using CLOUD = pcl::PointCloud<POINT>;
  using CLOUD_PTR = CLOUD::Ptr;

 public:
  CloudData()
      :cloud_ptr(new CLOUD()) {
  }

 public:
  double time = 0.0;
  CLOUD_PTR cloud_ptr;
};
}


#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_CLOUD_DATA_H_
