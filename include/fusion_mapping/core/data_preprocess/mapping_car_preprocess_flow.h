//
// Created by linsin on 24/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_DATA_PREPROCESS_MAPPING_CAR_PREPROCESS_FLOW_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_DATA_PREPROCESS_MAPPING_CAR_PREPROCESS_FLOW_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "fusion_mapping/core/subscriber/odometry_subscriber.h"
#include "fusion_mapping/core/publisher/imu_publisher.h"
#include "fusion_mapping/core/publisher/velocity_publisher.h"

namespace FM{
class MappingCarPreprocessFlow{
 public:
  MappingCarPreprocessFlow(ros::NodeHandle& nh);

  bool Run();
 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool PublishData();
  bool ConstructIMUData();
  bool ConstructVelocityData();

 private:
  // subscriber
  std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
  // publisher
  std::shared_ptr<IMUPublisher> imu_pub_ptr_;
  std::shared_ptr<VelocityPublisher> velo_sub_ptr_;

  std::deque<nav_msgs::Odometry> odom_data_buff_;

  nav_msgs::Odometry current_odom_data_;
  IMUData current_imu_data_;
  VelocityData current_velo_data_;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_DATA_PREPROCESS_MAPPING_CAR_PREPROCESS_FLOW_H_
