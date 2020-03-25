//
// Created by linsin on 24/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_VELOCITY_PUBLISHER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_VELOCITY_PUBLISHER_H_
#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "fusion_mapping/core/sensor_data/velocity_data.h"

namespace FM{
class VelocityPublisher{
 public:
  VelocityPublisher(ros::NodeHandle &nh,
      std::string topic_name,
      std::string frame_id,
      size_t buff_size);
  VelocityPublisher() = default;
  void Publish(VelocityData velocity_data);
  void Publish(VelocityData velocity_data, double time);
  void PublishData(VelocityData velocity_data, ros::Time time);
  bool HasSubscribers();
 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
  geometry_msgs::TwistStamped velocity_msg_;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_VELOCITY_PUBLISHER_H_
