//
// Created by linsin on 13/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_PUBLISHER_CLOUD_PUBLISHER_H_
#define FUSION_MAPPING_INCLUDE_PUBLISHER_CLOUD_PUBLISHER_H_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "fusion_mapping/core/sensor_data/cloud_data.h"

namespace FM {
class CloudPublisher {
 public:
  CloudPublisher(ros::NodeHandle& nh,
                 std::string topic_name,
                 std::string frame_id,
                 size_t buff_size);
  CloudPublisher() = default;

  void Publish(CloudData::CLOUD_PTR& cloud_ptr_input, double time);
  void Publish(CloudData::CLOUD_PTR& cloud_ptr_input);

  bool HasSubscribers();

 private:
  void PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time);

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};
}
#endif //FUSION_MAPPING_INCLUDE_PUBLISHER_CLOUD_PUBLISHER_H_
