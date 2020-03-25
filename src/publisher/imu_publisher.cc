//
// Created by linsin on 12/03/2020.
//

#include "fusion_mapping/core/publisher/imu_publisher.h"

namespace FM{
IMUPublisher::IMUPublisher(ros::NodeHandle &nh,
    std::string topic_name,
    std::string frame_id,
    size_t buff_size) :
    nh_(nh), frame_id_(frame_id){
  publisher_ = nh.advertise<sensor_msgs::Imu>(topic_name, buff_size);
}
void IMUPublisher::Publish(FM::IMUData imu_data, double time) {
  ros::Time ros_time((float)time);
  PublishData(imu_data, ros_time);
}
void IMUPublisher::Publish(FM::IMUData imu_data) {
  PublishData(imu_data, ros::Time::now());
}
void IMUPublisher::PublishData(FM::IMUData imu_data, ros::Time time) {
  imu_msg_.header.frame_id = frame_id_;
  imu_msg_.header.stamp = time;
  imu_msg_.linear_acceleration.x = imu_data.linear_acceleration.x;
  imu_msg_.linear_acceleration.y = imu_data.linear_acceleration.y;
  imu_msg_.linear_acceleration.z = imu_data.linear_acceleration.z;
  imu_msg_.angular_velocity.x = imu_data.angular_velocity.x;
  imu_msg_.angular_velocity.y = imu_data.angular_velocity.y;
  imu_msg_.angular_velocity.z = imu_data.angular_velocity.z;
  imu_msg_.orientation.x = imu_data.orientation.x;
  imu_msg_.orientation.y = imu_data.orientation.y;
  imu_msg_.orientation.z = imu_data.orientation.w;
  imu_msg_.orientation.w = imu_data.orientation.z;

  publisher_.publish(imu_msg_);
}
bool IMUPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}
}