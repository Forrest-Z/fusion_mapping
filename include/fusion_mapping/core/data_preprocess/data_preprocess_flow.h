//
// Created by linsin on 12/03/2020.
//

/*
 * Function:
 *  1. receive sensor data
 *  2. multi sensors time align
 *  3. point cloud distortion
 *  4. sensor coordinate system align
 * Input:
 *  1. GNSS position, pose, angle and line velocity
 *  2. point cloud
 *  3. LIDAR to IMU transform matrix
 * Output:
 *  1. GNSS position and pose
 *  2. de-distortion point cloud
 */

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_DATA_PREPROCESS_DATA_PREPROCESS_FLOW_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_DATA_PREPROCESS_DATA_PREPROCESS_FLOW_H_
#include <ros/ros.h>

#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/subscriber/imu_subscriber.h"
#include "fusion_mapping/core/subscriber/gnss_subscriber.h"
#include "fusion_mapping/core/tf_listener/tf_listener.h"
#include "fusion_mapping/core/publisher/cloud_publisher.h"
#include "fusion_mapping/core/publisher/odometry_publisher.h"
#include "fusion_mapping/core/front_end/front_end.h"
#include "fusion_mapping/core/param/front_param.h"
#include "fusion_mapping/core/sensor_data/velocity_data.h"
#include "fusion_mapping/core/subscriber/velocity_subscriber.h"
#include "fusion_mapping/core/tools/file_manager.h"
#include "fusion_mapping/core/global_defination/global_defination.h"
#include "fusion_mapping/core/models/scan_adjust/distortion_adjust.h"

namespace FM{
class DataPreprocessFlow {
 public:
  DataPreprocessFlow(ros::NodeHandle& nh);

  bool Run();

 private:
  bool ReadData();
  bool InitCalibration();
  bool InitGNSS();
  bool HasData();
  bool ValidData();
  bool TransformData();
  bool PublishData();

 private:
  // subscriber
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
  std::shared_ptr<TFListener> lidar_to_imu_ptr_;
  // publisher
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
  // models
  std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

  Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

  std::deque<CloudData> cloud_data_buff_;
  std::deque<IMUData> imu_data_buff_;
  std::deque<VelocityData> velocity_data_buff_;
  std::deque<GNSSData> gnss_data_buff_;

  CloudData current_cloud_data_;
  IMUData current_imu_data_;
  VelocityData current_velocity_data_;
  GNSSData current_gnss_data_;

  Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_DATA_PREPROCESS_DATA_PREPROCESS_FLOW_H_
