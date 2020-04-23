//
// Created by linsin on 23/04/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_CALIBRATION_COMPONENT_LIDAR_TO_VEHICLE_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_CALIBRATION_COMPONENT_LIDAR_TO_VEHICLE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace FM{
class LidarToVehicle{
 public:
  Eigen::Matrix3d calibrationRollPitchWithNormal(Eigen::Vector3d& ground_normal, Eigen::Vector3d& std_normal);
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_CALIBRATION_COMPONENT_LIDAR_TO_VEHICLE_H_
