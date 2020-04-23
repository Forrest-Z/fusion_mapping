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
  int num_ground_iterator_ = 20; // 优化次数
  int num_lpr_ = 10;  // 选取的lpr个数

  double sensor_height_ = 0.6;
  double ground_seeds_threshold_ = 0.5; // 用于选取种子点的阈值，当点云内的点的高度小于LPR的高度加上此阈值时，我们将该点加入种子点集
  double ground_distance_threshold_ = 0.1; // 平面距离阈值，我们会计算点云中每一个点到我们拟合的平面的正交投影的距离，而这个平面距离阈值，就是用来判定点是否属于地面

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
