//
// Created by linsin on 24/03/2020.
//

#include "fusion_mapping/core/publisher/velocity_publisher.h"

namespace FM{
VelocityPublisher::VelocityPublisher(ros::NodeHandle &nh,
                                     std::string topic_name,
                                     std::string frame_id,
                                     size_t buff_size):nh_(nh), frame_id_(frame_id){
  publisher_ = nh.advertise<geometry_msgs::TwistStamped>(topic_name, buff_size);
}
void VelocityPublisher::Publish(FM::VelocityData velocity_data, double time) {
  ros::Time ros_time((float)time);
  PublishData(velocity_data, ros_time);
}
void VelocityPublisher::Publish(FM::VelocityData velocity_data) {
  PublishData(velocity_data, ros::Time::now());
}
void VelocityPublisher::PublishData(FM::VelocityData velocity_data, ros::Time time) {
  velocity_msg_.header.frame_id = frame_id_;
  velocity_msg_.header.stamp = time;
  velocity_msg_.twist.linear.x = velocity_data.linear_velocity.x;
  velocity_msg_.twist.linear.y = velocity_data.linear_velocity.y;
  velocity_msg_.twist.linear.z = velocity_data.linear_velocity.z;
  velocity_msg_.twist.angular.x = velocity_data.angular_velocity.x;
  velocity_msg_.twist.angular.y = velocity_data.angular_velocity.y;
  velocity_msg_.twist.angular.z = velocity_data.angular_velocity.z;
  publisher_.publish(velocity_msg_);
}
bool VelocityPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}
}
