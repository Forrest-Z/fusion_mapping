//
// Created by linsin on 08/04/2020.
//

#include "fusion_mapping/core/calibration/lidar_to_vehicle/lidar_to_vehicle_flow.h"

namespace FM{
LidarToVehicleFlow::LidarToVehicleFlow(ros::NodeHandle &nh, std::string cloud_topic, std::string odom_topic, std::string labeled_topic) {
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
  odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, odom_topic, 100000);

  cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, labeled_topic, "/points", 100000);
}
bool LidarToVehicleFlow::Run() {
  if(!ReadData()) return false;
  while(HasData()) {
    if(!ValidData()) continue;
    if(CalibrateData()) {
      PublishData();
    }
  }
}

bool LidarToVehicleFlow::ReadData() {
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  return true;
}

bool LidarToVehicleFlow::HasData() {
  return cloud_data_buff_.size() > 0;
}

bool LidarToVehicleFlow::CalibrateData() {
  current_cloud_data_ = cloud_data_buff_.front();
  cloud_data_buff_.pop_front();

  return true;
}

bool LidarToVehicleFlow::CalibrateData() {
  lidar_to_vehicle_ptr_
}

}