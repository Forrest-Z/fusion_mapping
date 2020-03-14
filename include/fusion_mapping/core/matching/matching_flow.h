//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MATCHING_MATCHING_FLOW_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MATCHING_MATCHING_FLOW_H_
#include <ros/ros.h>
#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/subscriber/odometry_subscriber.h"
#include "fusion_mapping/core/publisher/cloud_publisher.h"
#include "fusion_mapping/core/publisher/odometry_publisher.h"
#include "fusion_mapping/core/publisher/tf_broadcaster.h"
#include "fusion_mapping/core/matching/matching.h"

namespace FM {
class MatchingFlow {
 public:
  MatchingFlow(ros::NodeHandle& nh);

  bool Run();

 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateMatching();
  bool PublishData();

 private:
  // subscriber
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
  // publisher
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
  std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
  std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;
  // matching
  std::shared_ptr<Matching> matching_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> gnss_data_buff_;

  CloudData current_cloud_data_;
  PoseData current_gnss_data_;

  Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MATCHING_MATCHING_FLOW_H_
