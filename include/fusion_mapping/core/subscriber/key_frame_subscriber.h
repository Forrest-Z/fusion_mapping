//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_H_
#include <deque>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "fusion_mapping/core/sensor_data/key_frame.h"

namespace FM {
class KeyFrameSubscriber {
 public:
  KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  KeyFrameSubscriber() = default;
  void ParseData(std::deque<KeyFrame>& key_frame_buff);

 private:
  void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& key_frame_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<KeyFrame> new_key_frame_;
  std::mutex buff_mutex_;
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_H_
