//
// Created by linsin on 16/04/2020.
//

#include "fusion_mapping/core/models/camera/rgbd/rgbd_base.h"

namespace FM{
pcl::PointCloud<pcl::PointXYZRGBA> RGBDBase::getPointCloud(cv::Mat &rgb, cv::Mat &depth) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  int row_si = depth.rows;
  int col_si = depth.cols;
  pcl::PointXYZRGBA p;
  for(int m = 0; m < row_si; ++m) {
    for(int n = 0; n < col_si; ++n) {
      ushort d = depth.ptr<ushort>(m)[n]; // get depth value
      if(d == 0) continue;
      p.z = double(d) / camera_factor_;
      p.x = (n - camera_cx_) * p.z / camera_fx_;
      p.y = (m - camera_cy_) * p.z / camera_fy_;
      p.b = rgb.ptr<uchar>(m)[n*3]; // three channel
      p.g = rgb.ptr<uchar>(m)[n*3 + 1];
      p.r = rgb.ptr<uchar>(m)[n*3 + 2];
      cloud->push_back(p);
    }
    cloud->height = 1;
    cloud->width = cloud->size();
    cloud->is_dense = false;
    return *cloud;
  }
}

cv::Point3f RGBDBase::point2DTo3D(cv::Point3f &point) {
  cv::Point3f p;
  p.z = double( point.z ) / camera_factor_;
  p.x = ( point.x - camera_cx_) * p.z / camera_fx_;
  p.y = ( point.y - camera_cy_) * p.z / camera_fy_;
  return p;
}
}