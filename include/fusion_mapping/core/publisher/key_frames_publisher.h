//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_KEY_FRAMES_PUBLISHER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_KEY_FRAMES_PUBLISHER_H_
#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "fusion_mapping/core/sensor_data/key_frame.h"

namespace FM {
class KeyFramesPublisher {
 public:
  KeyFramesPublisher(ros::NodeHandle& nh,
                     std::string topic_name,
                     std::string frame_id,
                     int buff_size);
  KeyFramesPublisher() = default;

  void Publish(const std::deque<KeyFrame>& key_frames);

  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_ = "";
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_KEY_FRAMES_PUBLISHER_H_
