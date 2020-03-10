//
// Created by linsin on 10/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_POINT_TYPE_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_POINT_TYPE_H_
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>



namespace FM{
enum Label {
  UNLABEL,
  GROUND,
  OTHER
};

struct PointXYZILR {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t label;
  int16_t ring;
  double timestamp;
  int azimuth;
  inline PointXYZILR() {
    x = y = z = 0;
    intensity = 0;
    label = Label::UNLABEL;
    ring = -1;
    azimuth = -1;
    timestamp = 0;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

}

POINT_CLOUD_REGISTER_POINT_STRUCT(FM::PointXYZILR,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(uint16_t, label, label)
(int16_t , ring , ring)
(int , azimuth , azimuth)
(double , timestamp, timestamp)
)
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SENSOR_DATA_POINT_TYPE_H_
