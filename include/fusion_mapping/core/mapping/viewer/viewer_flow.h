//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_VIEWER_VIEWER_FLOW_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_VIEWER_VIEWER_FLOW_H_
#include <deque>
#include <ros/ros.h>
// subscriber
#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/subscriber/odometry_subscriber.h"
#include "fusion_mapping/core/subscriber/key_frame_subscriber.h"
#include "fusion_mapping/core/subscriber/key_frames_subscriber.h"
// publisher
#include "fusion_mapping/core/publisher/odometry_publisher.h"
#include "fusion_mapping/core/publisher/cloud_publisher.h"
// viewer
#include "fusion_mapping/core/mapping/viewer/viewer.h"


namespace FM {
class ViewerFlow {
 public:
  ViewerFlow(ros::NodeHandle& nh);

  bool Run();
  bool SaveMap();

 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateViewer();
  bool PublishData();

 private:
  // subscriber
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
  std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
  std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;
  // publisher
  std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
  std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
  std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
  std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
  // viewer
  std::shared_ptr<Viewer> viewer_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> transformed_odom_buff_;
  std::deque<KeyFrame> key_frame_buff_;
  std::deque<KeyFrame> optimized_key_frames_;
  std::deque<KeyFrame> all_key_frames_;

  CloudData current_cloud_data_;
  PoseData current_transformed_odom_;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_VIEWER_VIEWER_FLOW_H_
