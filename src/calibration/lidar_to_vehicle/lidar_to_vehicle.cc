//
// Created by linsin on 08/04/2020.
//
#include "fusion_mapping/core/calibration/lidar_to_vehicle/lidar_to_vehicle.h"

namespace FM{
bool LidarToVehicle::Calibration6DOF(FM::CloudData &raw_cloud) {
  ground_feature_ptr_ = std::make_shared<GroundFeature>();

}
}
