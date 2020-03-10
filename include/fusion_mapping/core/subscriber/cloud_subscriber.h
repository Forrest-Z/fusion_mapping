//
// Created by linsin on 13/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_SUBSCRIBER_CLOUD_SUBSCRIBER_H_
#define FUSION_MAPPING_INCLUDE_SUBSCRIBER_CLOUD_SUBSCRIBER_H_

#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "fusion_mapping/core/sensor_data/cloud_data.h"

namespace FM{
class CloudSubscriber {
 public:
  CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  CloudSubscriber() = default;
  void ParseData(std::deque<CloudData>& deque_cloud_data);

 private:
  void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<CloudData> new_cloud_data_;
};
}


#endif //FUSION_MAPPING_INCLUDE_SUBSCRIBER_CLOUD_SUBSCRIBER_H_
