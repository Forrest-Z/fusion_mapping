//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_SRC_PUBLISHER_KEY_FRAME_PUBLISHER_H_
#define FUSION_MAPPING_SRC_PUBLISHER_KEY_FRAME_PUBLISHER_H_
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "fusion_mapping/core/sensor_data/key_frame.h"

namespace FM {
class KeyFramePublisher {
 public:
  KeyFramePublisher(ros::NodeHandle& nh,
                    std::string topic_name,
                    std::string frame_id,
                    int buff_size);
  KeyFramePublisher() = default;

  void Publish(KeyFrame& key_frame);

  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_ = "";
};
}
#endif //FUSION_MAPPING_SRC_PUBLISHER_KEY_FRAME_PUBLISHER_H_
