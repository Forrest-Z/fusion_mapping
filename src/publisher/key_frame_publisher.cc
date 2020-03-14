//
// Created by linsin on 12/03/2020.
//

#include "fusion_mapping/core/publisher/key_frame_publisher.h"

#include <Eigen/Dense>

namespace FM {
KeyFramePublisher::KeyFramePublisher(ros::NodeHandle& nh,
                                     std::string topic_name,
                                     std::string frame_id,
                                     int buff_size)
    :nh_(nh), frame_id_(frame_id) {

  publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, buff_size);
}

void KeyFramePublisher::Publish(KeyFrame& key_frame) {
  geometry_msgs::PoseWithCovarianceStamped pose_stamped;

  ros::Time ros_time((float)key_frame.time);
  pose_stamped.header.stamp = ros_time;
  pose_stamped.header.frame_id = frame_id_;

  pose_stamped.pose.pose.position.x = key_frame.pose(0,3);
  pose_stamped.pose.pose.position.y = key_frame.pose(1,3);
  pose_stamped.pose.pose.position.z = key_frame.pose(2,3);

  Eigen::Quaternionf q = key_frame.GetQuaternion();
  pose_stamped.pose.pose.orientation.x = q.x();
  pose_stamped.pose.pose.orientation.y = q.y();
  pose_stamped.pose.pose.orientation.z = q.z();
  pose_stamped.pose.pose.orientation.w = q.w();

  pose_stamped.pose.covariance[0] = (double)key_frame.index;

  publisher_.publish(pose_stamped);
}

bool KeyFramePublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}
}