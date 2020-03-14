//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_LOOP_POSE_PUBLISHER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_LOOP_POSE_PUBLISHER_H_
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "fusion_mapping/core/sensor_data/loop_pose.h"

namespace FM {
class LoopPosePublisher {
 public:
  LoopPosePublisher(ros::NodeHandle& nh,
                    std::string topic_name,
                    std::string frame_id,
                    int buff_size);
  LoopPosePublisher() = default;

  void Publish(LoopPose& loop_pose);

  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_ = "";
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_LOOP_POSE_PUBLISHER_H_
