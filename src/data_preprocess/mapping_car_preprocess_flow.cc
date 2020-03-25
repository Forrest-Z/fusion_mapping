//
// Created by linsin on 24/03/2020.
//

#include "fusion_mapping/core/data_preprocess/mapping_car_preprocess_flow.h"
namespace FM{
MappingCarPreprocessFlow::MappingCarPreprocessFlow(ros::NodeHandle &nh) {
  //subscriber
  odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/cpt/ins_odom", 100);
  //publisher
  imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/kitti/oxts/imu", "imu_link", 100);
  velo_sub_ptr_ = std::make_shared<VelocityPublisher>(nh, "/kitti/oxts/gps/vel", "imu_link", 100);
}
bool MappingCarPreprocessFlow::Run() {
  if(!ReadData())
    return false;
  while(HasData()) {
    if(!ValidData()) continue;
    ConstructIMUData();
    ConstructVelocityData();
    PublishData();
  }
}
bool MappingCarPreprocessFlow::ReadData() {
  odom_sub_ptr_->ParseData(odom_data_buff_);
  if(odom_data_buff_.size() == 0) return false;
  return true;
}
bool MappingCarPreprocessFlow::HasData() {
  if(odom_data_buff_.size() == 0) return false;
  return true;
}
bool MappingCarPreprocessFlow::ValidData() {
  current_odom_data_ = odom_data_buff_.front();
  odom_data_buff_.pop_front();
  return true;
}
bool MappingCarPreprocessFlow::ConstructIMUData() {
  current_imu_data_.linear_acceleration.x = current_odom_data_.twist.twist.linear.x;
  current_imu_data_.linear_acceleration.y = current_odom_data_.twist.twist.linear.y;
  current_imu_data_.linear_acceleration.z = current_odom_data_.twist.twist.linear.z;

  current_imu_data_.angular_velocity.x = current_odom_data_.twist.twist.angular.x;
  current_imu_data_.angular_velocity.y = current_odom_data_.twist.twist.angular.y;
  current_imu_data_.angular_velocity.z = current_odom_data_.twist.twist.angular.z;

  current_imu_data_.orientation.x = current_odom_data_.pose.pose.orientation.x;
  current_imu_data_.orientation.y = current_odom_data_.pose.pose.orientation.y;
  current_imu_data_.orientation.z = current_odom_data_.pose.pose.orientation.z;
  current_imu_data_.orientation.w = current_odom_data_.pose.pose.orientation.w;
  return true;
}
bool MappingCarPreprocessFlow::ConstructVelocityData() {
  current_velo_data_.linear_velocity.x = current_odom_data_.twist.twist.linear.x;
  current_velo_data_.linear_velocity.y = current_odom_data_.twist.twist.linear.y;
  current_velo_data_.linear_velocity.z = current_odom_data_.twist.twist.linear.z;
  current_velo_data_.angular_velocity.x = current_odom_data_.twist.twist.angular.x;
  current_velo_data_.angular_velocity.y = current_odom_data_.twist.twist.angular.y;
  current_velo_data_.angular_velocity.z = current_odom_data_.twist.twist.angular.z;
  return true;
}
bool MappingCarPreprocessFlow::PublishData() {
  imu_pub_ptr_->PublishData(current_imu_data_, current_odom_data_.header.stamp);
  velo_sub_ptr_->PublishData(current_velo_data_, current_odom_data_.header.stamp);
  return true;
}
}