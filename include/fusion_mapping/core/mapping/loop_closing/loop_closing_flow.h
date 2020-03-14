//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_H_
#include <deque>
#include <ros/ros.h>
#include "fusion_mapping/core/subscriber/key_frame_subscriber.h"
#include "fusion_mapping/core/publisher/loop_pose_publisher.h"
#include "fusion_mapping/core/mapping/loop_closing/loop_closing.h"
#include "fusion_mapping/core/global_defination/global_defination.h"

namespace FM {
class LoopClosingFlow {
 public:
  LoopClosingFlow(ros::NodeHandle& nh);

  bool Run();

 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool PublishData();

 private:
  // subscriber
  std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
  std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;
  // publisher
  std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
  // loop closing
  std::shared_ptr<LoopClosing> loop_closing_ptr_;

  std::deque<KeyFrame> key_frame_buff_;
  std::deque<KeyFrame> key_gnss_buff_;

  KeyFrame current_key_frame_;
  KeyFrame current_key_gnss_;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_H_
