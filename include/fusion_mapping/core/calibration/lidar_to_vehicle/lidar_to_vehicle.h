//
// Created by linsin on 08/04/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_CALIBRATION_LIDAR_TO_VEHICLE_LIDAR_TO_VEHICLE_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_CALIBRATION_LIDAR_TO_VEHICLE_LIDAR_TO_VEHICLE_H_

#include "fusion_mapping/core/sensor_data/cloud_data.h"
#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/models/feature/ground_feature.h"

namespace FM{
class LidarToVehicle{
 public:
  LidarToVehicle();
  bool Calibration6DOF(CloudData& raw_cloud);

 private:
  bool CalibrationRollPitch(CloudData& raw_cloud);

 private:
  std::shared_ptr<GroundFeature> ground_feature_ptr_;
};
};
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_CALIBRATION_LIDAR_TO_VEHICLE_LIDAR_TO_VEHICLE_H_
