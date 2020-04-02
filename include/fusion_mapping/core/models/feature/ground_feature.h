//
// Created by linsin on 31/03/2020.
//
// Paper-related:
// connected component labelling problem
// Fast Segmentation of 3D Point Clouds: A Paradigm on LiDAR Data for Autonomous Vehicle Applications
// Suzuki, A run-based two-scan labeling algorithm, TIP 2008. 提出的方法. L. He在这方面有好几篇文章
#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_FEATURE_GROUND_FEATURE_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_FEATURE_GROUND_FEATURE_H_

#include <pcl/point_types.h>
#include <algorithm>
#include <vector>
#include <Eigen/Core>
#include <pcl/pcl_base.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace FM{
typedef pcl::PointXYZI PointTypeGround;
class GroundFeature{
 public:
  pcl::PointCloud<PointTypeGround> groundFeature(pcl::PointCloud<PointTypeGround>& raw_points);

 private:
  pcl::PointCloud<PointTypeGround> total_ground;
  pcl::PointCloud<PointTypeGround> fragment_points_[20][20];

  double x_low_limitation_[20];
  double y_low_limitation_[20];

  int x_fragment_number_ = 5, y_fragment_number_ = 5;
  int num_ground_iterator_ = 10;
  int num_lpr_ = 20;

  double sensor_height_ = 0.6;
  double ground_seeds_threshold_ = 1.2;
  double ground_distance_threshold_ = 0.3;

  float d_;
  Eigen::MatrixXf normal_;
  float th_dist_d_;
  double closed_thres_;

  pcl::PointCloud<PointTypeGround>::Ptr g_seeds_pc_;
  pcl::PointCloud<PointTypeGround>::Ptr g_ground_pc_;
  pcl::PointCloud<PointTypeGround>::Ptr g_non_ground_pc_;

 private:
  void SeparateGround(pcl::PointCloud<PointTypeGround>& all_points);
  void SinglePartExtract(pcl::PointCloud<PointTypeGround>& single_points);
  void ExtractInitialSeeds(pcl::PointCloud<PointTypeGround>& process_cloud);
  void EstimatePlane();
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_FEATURE_GROUND_FEATURE_H_
