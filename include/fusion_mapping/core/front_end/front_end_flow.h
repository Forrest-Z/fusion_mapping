//
// Created by linsin on 17/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FRONT_END_FRONT_END_FLOW_H_
#define FUSION_MAPPING_INCLUDE_FRONT_END_FRONT_END_FLOW_H_

#include <ros/ros.h>

#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/subscriber/imu_subscriber.h"
#include "fusion_mapping/core/subscriber/gnss_subscriber.h"
#include "fusion_mapping/core/tf_listener/tf_listener.h"
#include "fusion_mapping/core/publisher/cloud_publisher.h"
#include "fusion_mapping/core/publisher/odometry_publisher.h"
#include "fusion_mapping/core/front_end/front_end.h"
#include "fusion_mapping/core/param/front_param.h"

namespace FM{
class FrontEndFlow {
 public:
  FrontEndFlow(ros::NodeHandle& nh);

  bool Run();
  bool SaveMap();
  bool PublishGlobalMap();

 private:
  bool ReadData();
  bool InitCalibration();
  bool InitGNSS();
  bool HasData();
  bool ValidData();
  bool UpdateGNSSOdometry();
  bool UpdateLaserOdometry();
  bool PublishData();

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
  std::shared_ptr<TFListener> lidar_to_imu_ptr_;
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
  std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
  std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  std::deque<IMUData> imu_data_buff_;
  std::deque<GNSSData> gnss_data_buff_;
  Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
  CloudData current_cloud_data_;
  IMUData current_imu_data_;
  GNSSData current_gnss_data_;

  CloudData::CLOUD_PTR local_map_ptr_;
  CloudData::CLOUD_PTR global_map_ptr_;
  CloudData::CLOUD_PTR current_scan_ptr_;
  Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}


#endif //FUSION_MAPPING_INCLUDE_FRONT_END_FRONT_END_FLOW_H_
