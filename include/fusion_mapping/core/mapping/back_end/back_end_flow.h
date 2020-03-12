//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_BACK_END_BACK_END_FLOW_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_BACK_END_BACK_END_FLOW_H_
#include <ros/ros.h>

#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/subscriber/odometry_subscriber.h"
#include "fusion_mapping/core/publisher/odometry_publisher.h"
#include "fusion_mapping/core/publisher/key_frame_publisher.h"
#include "fusion_mapping/core/publisher/key_frames_publisher.h"
#include "fusion_mapping/core/mapping/back_end/back_end.h"

namespace FM {
class BackEndFlow {
 public:
  BackEndFlow(ros::NodeHandle& nh);

  bool Run();

 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateBackEnd();
  bool SaveTrajectory();
  bool PublishData();

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;

  std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
  std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
  std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
  std::shared_ptr<BackEnd> back_end_ptr_;

  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> gnss_pose_data_buff_;
  std::deque<PoseData> laser_odom_data_buff_;

  PoseData current_gnss_pose_data_;
  PoseData current_laser_odom_data_;
  CloudData current_cloud_data_;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_BACK_END_BACK_END_FLOW_H_
