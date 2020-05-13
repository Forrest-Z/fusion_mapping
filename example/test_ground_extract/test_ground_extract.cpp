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
#include "fusion_mapping/core/calibration/component/lidar_to_vehicle.h"

using namespace FM;
ros::Publisher ground_pub;
Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
Eigen::Matrix4d global_trans = Eigen::Matrix4d::Identity();
bool roll_pitch_calibrated = false;
void Callback(const sensor_msgs::PointCloud2ConstPtr& input_ros) {
//  roll_pitch_calibrated = true;
  pcl::PointCloud<pcl::PointXYZI> pcl_input, ground_pcl;
  pcl::fromROSMsg(*input_ros, pcl_input);
  GroundFeature ground_feature;
  pcl_input.height = 1;
  pcl_input.width = pcl_input.size();
  ground_feature.groundFeature(pcl_input);
  Eigen::Vector3d ground_normal = Eigen::Vector3d::Identity(), pre_ground_normal;
  int si = pcl_input.size();
  for(int i = 0 ; i < si ; ++i) {
    if(pcl_input.points.at(i).intensity == 0) ground_pcl.push_back(pcl_input.at(i));
  }
  double dis = 100;
  Eigen::Vector3d std_normal(0 , 0 , 1);
  pcl::transformPointCloud(ground_pcl, ground_pcl, global_trans);
  while(dis > 0.0001) {
    if(dis != 100) pre_ground_normal = ground_normal;
    else pre_ground_normal = std_normal;
    ground_feature.groundWithSAC(ground_pcl, ground_normal);
    LidarToVehicle lidar_to_vehicle;
    trans.block<3,3>(0,0) = lidar_to_vehicle.calibrationRollPitchWithNormal(ground_normal, std_normal);
    pcl::transformPointCloud(ground_pcl, ground_pcl, trans);
    dis = Eigen::Vector3d(pre_ground_normal - ground_normal).norm();
    global_trans = trans * global_trans;
    std::cout << "distance between two normal: " << dis << std::endl;
  }
  std::cout << "global trans is \n" << global_trans << std::endl;
  Eigen::Quaterniond global_qua(global_trans.block<3, 3>(0 , 0));
  std::cout << "global qua is \n" << global_qua.x() << " " <<global_qua.y() << " "<<global_qua.z() << " "<<global_qua.w() << " \n";
  pcl_input = ground_pcl;
  std::cout << "ground_normal:\n" << ground_normal << "\n****************************" << std::endl;
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