//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "fusion_mapping/core/sensor_data/velocity_data.h"

namespace FM {
class VelocitySubscriber {
 public:
  VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  VelocitySubscriber() = default;
  void ParseData(std::deque<VelocityData>& deque_velocity_data);

 private:
  void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<VelocityData> new_velocity_data_;
  std::mutex buff_mutex_;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_VELOCITY_SUBSCRIBER_H_
