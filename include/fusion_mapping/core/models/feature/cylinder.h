//
// Created by linsin on 18/04/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_FEATURE_CYLINDER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_FEATURE_CYLINDER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

namespace FM{
struct CylinderCoefficient{
  float x, y, z, l, m, n, r;
};

class CylinderRelated{
 public:
  CylinderCoefficient CylinderFitting(pcl::PointCloud<pcl::PointXYZ>& cloud);
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_FEATURE_CYLINDER_H_
