//
// Created by linsin on 16/04/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CAMERA_RGBD_RGBD_BASE_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CAMERA_RGBD_RGBD_BASE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <string>

namespace FM{
class RGBDBase{
 public:
  RGBDBase() = default;
  RGBDBase(double camera_factor, double camera_cx, double camera_cy, double camera_fx, double camera_fy):
  camera_factor_(camera_factor), camera_cx_(camera_cx), camera_cy_(camera_cy), camera_fx_(camera_fx), camera_fy_(camera_fy) {};
  pcl::PointCloud<pcl::PointXYZRGBA> getPointCloud(cv::Mat& rgb, cv::Mat& depth);
  cv::Point3f point2DTo3D(cv::Point3f& point);

 public:
  double camera_factor_ = 1000;
  double camera_cx_ = 325.5;
  double camera_cy_ = 235.5;
  double camera_fx_ = 518.0;
  double camera_fy_ = 519.0;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_CAMERA_RGBD_RGBD_BASE_H_
