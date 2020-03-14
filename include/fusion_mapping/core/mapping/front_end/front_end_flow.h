//
// Created by linsin on 12/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_FRONT_END_FRONT_END_FLOW_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_FRONT_END_FRONT_END_FLOW_H_
#include <ros/ros.h>

#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/publisher/odometry_publisher.h"
#include "fusion_mapping/core/mapping/front_end/front_end.h"

namespace FM {
class FrontEndFlow {
 public:
  FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic);

  bool Run();

 private:
  bool ReadData();
  bool HasData();
  bool ValidData();
  bool UpdateLaserOdometry();
  bool PublishData();

 private:
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
  std::shared_ptr<FrontEnd> front_end_ptr_;

  std::deque<CloudData> cloud_data_buff_;

  CloudData current_cloud_data_;

  Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_FRONT_END_FRONT_END_FLOW_H_
