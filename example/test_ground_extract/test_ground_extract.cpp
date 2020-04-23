//
// Created by linsin on 12/03/2020.
//

#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "glog/logging.h"
#include "models/feature/ground_feature.h"
#include "fusion_mapping/core/global_defination/global_defination.h"

using namespace FM;
ros::Publisher ground_pub;
void Callback(const sensor_msgs::PointCloud2ConstPtr& input_ros) {
  pcl::PointCloud<pcl::PointXYZI> pcl_input, ground_pcl;
  pcl::fromROSMsg(*input_ros, pcl_input);
  GroundFeature ground_feature;
  pcl_input.height = 1;
  pcl_input.width = pcl_input.size();
  ground_feature.groundFeature(pcl_input);
  Eigen::Vector3d ground_normal = Eigen::Vector3d::Identity();
  int si = pcl_input.size();
  for(int i = 0 ; i < si ; ++i) {
    if(pcl_input.points.at(i).intensity == 0) ground_pcl.push_back(pcl_input.at(i));
  }
  ground_feature.groundWithSAC(ground_pcl, ground_normal);
  pcl_input = ground_pcl;
  std::cout << "ground_normal:\n" << ground_normal << std::endl;
  sensor_msgs::PointCloud2 output_ros;
  pcl::toROSMsg(pcl_input, output_ros);
  output_ros.height = 1;
  output_ros.width = pcl_input.size();
  output_ros.is_dense = false;
  output_ros.header.stamp = ros::Time::now();
  output_ros.header.frame_id = "ground_extract";
  ground_pub.publish(output_ros);
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "test_ground_feature");
  ros::NodeHandle nh;
  ros::Subscriber cloud_sub = nh.subscribe("/velodyne_second/points_raw", 3, Callback);
  ground_pub = nh.advertise<sensor_msgs::PointCloud2>("ground_points", 3);

  ros::spin();
  return 0;
}