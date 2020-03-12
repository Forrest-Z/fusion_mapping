//
// Created by linsin on 14/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_PUBLISHER_ODEMETRY_PUBLISHER_H_
#define FUSION_MAPPING_INCLUDE_PUBLISHER_ODEMETRY_PUBLISHER_H_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace FM {
class OdometryPublisher {
 public:
  OdometryPublisher(ros::NodeHandle& nh,
                    std::string topic_name,
                    std::string base_frame_id,
                    std::string child_frame_id,
                    int buff_size);
  OdometryPublisher() = default;

  void Publish(const Eigen::Matrix4f& transform_matrix, double time);
  void Publish(const Eigen::Matrix4f& transform_matrix);

  bool HasSubscribers();

 private:
  void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

 private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  nav_msgs::Odometry odometry_;
};
}
#endif //FUSION_MAPPING_INCLUDE_PUBLISHER_ODEMETRY_PUBLISHER_H_
