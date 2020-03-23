//
// Created by linsin on 12/03/2020.
//

#include <iostream>

#include <vector>

#include <string.h> //包含strcmp的头文件,也可用: #include <ctring>

#include <dirent.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include "fusion_mapping/core/subscriber/cloud_subscriber.h"
#include "fusion_mapping/core/subscriber/gnss_subscriber.h"
#include "fusion_mapping/core/subscriber/odometry_subscriber.h"
#include "fusion_mapping/core/publisher/cloud_publisher.h"
#include "fusion_mapping/core/publisher/odometry_publisher.h"
#include "fusion_mapping/core/publisher/tf_broadcaster.h"
#include "fusion_mapping/core/matching/matching.h"
using namespace FM;
std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
std::deque<CloudData> cloud_data_buff_;
std::deque<PoseData> gnss_data_buff_;
std::vector<std::string> filename;
std::string path = "/home/linsin/fuck/road_points";
bool inited = false;
pcl::PointCloud<pcl::PointXYZI>::Ptr all_map(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI> tmp;
pcl::PointCloud<pcl::PointXYZI> current_pcd;
PoseData last, now;
bool ReadData() {
//  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  gnss_sub_ptr_->ParseData(gnss_data_buff_);
  return true;
}
bool Run() {
  ReadData();
  std::cout << filename.size() << std::endl;
  if(filename.size() < 5) {
    std::cout << "before sor\n";
    pcl::VoxelGrid<pcl::PointXYZI> sor;  //创建滤波对象
    sor.setInputCloud (all_map);            //设置需要过滤的点云给滤波对象
    sor.setLeafSize (0.5f, 0.5f, 0.5f);  //设置滤波时创建的体素体积为20cm的立方体
    sor.filter (tmp);           //执行滤波处理，存储输出
    all_map = nullptr;
    std::cout << "after sor\n";
    pcl::io::savePCDFileASCII("/home/linsin/fuck/fuck.pcd", tmp);
    exit(0);
  }
  if(!gnss_data_buff_.empty()) {
    GNSSData current_gnss;
    PoseData current_pose = gnss_data_buff_.front();
    gnss_data_buff_.pop_front();
//    current_gnss.latitude = current_pose.pose(0,3);
//    current_gnss.longitude = current_pose.pose(1,3);
//    current_gnss.altitude = current_pose.pose(2,3);
//    current_gnss.InitOriginPosition();
//    current_gnss.UpdateXYZ();
//    current_pose.pose(0, 3) = current_gnss.local_E;
//    current_pose.pose(1, 3) = current_gnss.local_N;
//    current_pose.pose(2, 3) = current_gnss.local_U;
    double local_time = current_pose.time - 632486048;
    double pcd_time = 0.0;
    std::string pcd_string_time = filename.back();
    std::string pcd_time_no_suf = "";
    int si = pcd_string_time.size();
    for(int i = 0 ; i < si - 4 ; ++i) {
      pcd_time_no_suf += pcd_string_time.at(i);
    }
    std::stringstream ss;
    ss << pcd_time_no_suf;
    ss >> pcd_time;
    printf("%lf\n" ,local_time);
    std::string str_time = std::to_string(local_time);
//    std::cout << pcd_time_no_suf << std::endl;
    if(fabs(pcd_time - local_time) < 0.05) {
      pcl::io::loadPCDFile<pcl::PointXYZI>(
          std::string(path + "/" + pcd_string_time), current_pcd);
      if(!inited) {
        inited = true;
        last = current_pose;
        *all_map = current_pcd;
      } else {
        Eigen::Matrix4f trans = last.pose.inverse() * current_pose.pose;
        std::cout << trans << std::endl;
        pcl::transformPointCloud(current_pcd, current_pcd, trans);
        *all_map += current_pcd;
      }
      filename.pop_back();
      std::cout << "ok!!!\n";
    }
  }
}
void getFileNames(const std::string path, std::vector<std::string>& filenames, const std::string suffix = "")
{

  DIR *pDir;

  struct dirent* ptr;

  if (!(pDir = opendir(path.c_str())))

    return;

  while ((ptr = readdir(pDir))!=0)

  {

    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)

    {

      std::string file = ptr->d_name;

      if (opendir(file.c_str()))

      {

        getFileNames(file, filenames, suffix);

      }

      else

      {

        if (suffix == file.substr(file.size() - suffix.size()))

        {

          filenames.push_back(file);

        }

      }

    }

  }

  closedir(pDir);

}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "merge_ground");

  ros::NodeHandle nh;
  gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/ins_linsin_odom", 1000000);
//  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/rslidar_points", 100000);
  getFileNames(path, filename);
  std::sort(filename.begin(), filename.end(), [](std::string a, std::string b) {return a > b;});
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    Run();

    rate.sleep();
  }
}

