//
// Created by linsin on 12/03/2020.
//

#include "fusion_mapping/core/models/camera/rgbd/rgbd_base.h"
#include <pcl/point_types.h>
#include <pcl/io/io.h>

int main(int argc, char** argv) {
  FM::RGBDBase tool;
  std::cout << "okk\n";

  //生成圆柱点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (float z(-10); z <= 10; z += 0.5) {
    for (float angle(0.0); angle <= 360.0; angle += 5.0) {
      pcl::PointXYZ basic_point;
      basic_point.x = 10+3.5*cos(angle / 180 * M_PI);
      basic_point.y = 200+3.5*sin(angle / 180 * M_PI);
      basic_point.z = z;
      cloud->points.push_back(basic_point);
    }
  } cloud->height = 1;
  cloud->width = cloud->size();
  cloud->is_dense = false;
  pcl::io::savePCDFileASCII("/home/linsin/fuck/cylinder.pcd", *cloud);
}

