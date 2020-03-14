//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_TF_BROADCASTER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_TF_BROADCASTER_H_
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace FM {
class TFBroadCaster {
 public:
  TFBroadCaster(std::string frame_id, std::string child_frame_id);
  TFBroadCaster() = default;
  void SendTransform(Eigen::Matrix4f pose, double time);
 protected:
  tf::StampedTransform transform_;
  tf::TransformBroadcaster broadcaster_;
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_TF_BROADCASTER_H_
