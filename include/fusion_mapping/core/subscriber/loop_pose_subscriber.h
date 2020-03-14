//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_H_

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "fusion_mapping/core/sensor_data/loop_pose.h"

namespace FM {
class LoopPoseSubscriber {
 public:
  LoopPoseSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  LoopPoseSubscriber() = default;
  void ParseData(std::deque<LoopPose>& loop_pose_buff);

 private:
  void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_pose_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<LoopPose> new_loop_pose_;
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_H_
