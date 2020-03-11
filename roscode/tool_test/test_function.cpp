//
// Created by linsin on 14/02/2020.
//

#include <ros/ros.h>
#include <pcl/common/transforms.h>

#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/subscriber/imu_subscriber.h"
#include "fusion_mapping/core/subscriber/gnss_subscriber.h"
#include "fusion_mapping/core/tf_listener/tf_listener.h"
#include "fusion_mapping/core/publisher/cloud_publisher.h"
#include "fusion_mapping/core/publisher/odometry_publisher.h"

using namespace FM;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_frame_node");
  ros::NodeHandle nh;

  std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
  std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
  std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
  std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

  std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
  std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

  std::deque<CloudData> cloud_data_buff;
  std::deque<IMUData> imu_data_buff;
  std::deque<GNSSData> gnss_data_buff;
  Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
  bool transform_received = false;
  bool gnss_origin_position_inited = false;

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    cloud_sub_ptr->ParseData(cloud_data_buff);
    imu_sub_ptr->ParseData(imu_data_buff);
    gnss_sub_ptr->ParseData(gnss_data_buff);

    if (!transform_received) {
      if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
        transform_received = true;
        // LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu;
      }
    } else {
      while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
        CloudData cloud_data = cloud_data_buff.front();
        IMUData imu_data = imu_data_buff.front();
        GNSSData gnss_data = gnss_data_buff.front();

        double d_time = cloud_data.time - imu_data.time;
        if (d_time < -0.05) {
          cloud_data_buff.pop_front();
        } else if (d_time > 0.05) {
          imu_data_buff.pop_front();
          gnss_data_buff.pop_front();
        } else {
          cloud_data_buff.pop_front();
          imu_data_buff.pop_front();
          gnss_data_buff.pop_front();

          Eigen::Matrix4f odometry_matrix;

          if (!gnss_origin_position_inited) {
            gnss_data.InitOriginPosition();
            gnss_origin_position_inited = true;
          }
          gnss_data.UpdateXYZ();
          odometry_matrix(0,3) = gnss_data.local_E;
          odometry_matrix(1,3) = gnss_data.local_N;
          odometry_matrix(2,3) = gnss_data.local_U;
          odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();
          odometry_matrix *= lidar_to_imu;

          pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, odometry_matrix);

          cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
          odom_pub_ptr->Publish(odometry_matrix);
        }
      }
    }

    rate.sleep();
  }

  return 0;
}