//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_ODOMETRY_SUBSCRIBER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_ODOMETRY_SUBSCRIBER_H_
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "fusion_mapping/core/sensor_data/pose_data.h"

namespace FM {
class OdometrySubscriber {
 public:
  OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  OdometrySubscriber() = default;
  void ParseData(std::deque<PoseData>& deque_pose_data);

 private:
  void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<PoseData> new_pose_data_;
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_SUBSCRIBER_ODOMETRY_SUBSCRIBER_H_
