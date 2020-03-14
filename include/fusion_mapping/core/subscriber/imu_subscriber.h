//
// Created by linsin on 14/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_SUBSCRIBER_IMU_SUBSCRIBER_H_
#define FUSION_MAPPING_INCLUDE_SUBSCRIBER_IMU_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "fusion_mapping/core/sensor_data/imu_data.h"

namespace FM {
class IMUSubscriber {
 public:
  IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  IMUSubscriber() = default;
  void ParseData(std::deque<IMUData>& deque_imu_data);

 private:
  void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<IMUData> new_imu_data_;
  std::mutex buff_mutex_;
};
}


#endif //FUSION_MAPPING_INCLUDE_SUBSCRIBER_IMU_SUBSCRIBER_H_
