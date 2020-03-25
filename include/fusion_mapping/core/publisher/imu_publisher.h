//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_IMU_PUBLISHER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_IMU_PUBLISHER_H_

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "fusion_mapping/core/sensor_data/imu_data.h"

namespace FM {
class IMUPublisher {
 public:
  IMUPublisher(ros::NodeHandle& nh,
               std::string topic_name,
               std::string frame_id,
               size_t buff_size);
  IMUPublisher() = default;

  void Publish(IMUData imu_data, double time);
  void Publish(IMUData imu_data);
  void PublishData(IMUData imu_data, ros::Time time);

  bool HasSubscribers();

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;

  sensor_msgs::Imu imu_msg_;
};
}
#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PUBLISHER_IMU_PUBLISHER_H_
