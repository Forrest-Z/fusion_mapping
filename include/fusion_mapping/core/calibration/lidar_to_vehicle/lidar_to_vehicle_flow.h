//
// Created by linsin on 08/04/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_CALIBRATION_LIDAR_TO_VEHICLE_LIDAR_TO_VEHICLE_FLOW_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_CALIBRATION_LIDAR_TO_VEHICLE_LIDAR_TO_VEHICLE_FLOW_H_

#include <ros/ros.h>

#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/subscriber/odometry_subscriber.h"
#include "fusion_mapping/core/publisher/cloud_publisher.h"
#include "fusion_mapping/core/calibration/lidar_to_vehicle/lidar_to_vehicle.h"

namespace FM{
class LidarToVehicleFlow{
 public:
  LidarToVehicleFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic, std::string labeled_topic);
  bool Run();
 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool CalibrateData();
  bool PublishData();

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;

  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> odom_data_buff_;

  CloudData current_cloud_data_;

  std::shared_ptr<LidarToVehicle> lidar_to_vehicle_ptr_;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_CALIBRATION_LIDAR_TO_VEHICLE_LIDAR_TO_VEHICLE_FLOW_H_
