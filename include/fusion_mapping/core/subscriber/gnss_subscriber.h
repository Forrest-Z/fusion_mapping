//
// Created by linsin on 13/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_SUBSCRIBER_GNSS_SUBSCRIBER_H_
#define FUSION_MAPPING_INCLUDE_SUBSCRIBER_GNSS_SUBSCRIBER_H_


#include <deque>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "fusion_mapping/core/sensor_data/gnss_data.h"

namespace FM {
class GNSSSubscriber {
 public:
  GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  GNSSSubscriber() = default;
  void ParseData(std::deque<GNSSData>& deque_gnss_data);

 private:
  void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<GNSSData> new_gnss_data_;
  std::mutex buff_mutex_;
};
}

#endif //FUSION_MAPPING_INCLUDE_SUBSCRIBER_GNSS_SUBSCRIBER_H_
