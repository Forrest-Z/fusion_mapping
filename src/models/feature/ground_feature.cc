//
// Created by linsin on 31/03/2020.
//

#include "fusion_mapping/core/models/feature/ground_feature.h"

namespace FM{
pcl::PointCloud<PointTypeGround> GroundFeature::groundFeature(pcl::PointCloud<FM::PointTypeGround> &raw_points) {
  g_ground_pc_.reset(new pcl::PointCloud<PointTypeGround>());
  g_non_ground_pc_.reset(new pcl::PointCloud<PointTypeGround>());
  g_seeds_pc_.reset(new pcl::PointCloud<PointTypeGround>());
  int si = raw_points.points.size();
  for(int i = 0 ; i < si ; ++i) {
    raw_points.points.at(i).intensity = 1;
  }

  SinglePartExtract(raw_points);
}
void GroundFeature::SeparateGround(pcl::PointCloud<FM::PointTypeGround> &all_points) {
  double max_x = 0.0, max_y = 0.0;
  double min_x = 123456.6, min_y = 123456.6;
  int si_all = all_points.size();
  for(int i = 0 ; i < si_all ; ++i) {
    max_x = std::max(max_x, double(all_points.at(i).x));
    max_y = std::max(max_y, double(all_points.at(i).y));
    min_x = std::min(min_x, double(all_points.at(i).x));
    min_y = std::min(min_y, double(all_points.at(i).y));
  } double x_length = max_x - min_x, y_length = max_y - min_y;
  double single_x = x_length / double(x_fragment_number_);
  double single_y = y_length / double(y_fragment_number_);
  for(int i = 0; i < x_fragment_number_; ++i) {
    x_low_limitation_[i + 1] = min_x + i * single_x;
  } for(int i = 0 ; i <= y_fragment_number_; ++i) {
    y_low_limitation_[i + 1] = min_y + i * single_y;
  } int si = all_points.size();
  for(int i = 0 ; i < si ; ++i) {
    PointTypeGround point = all_points.at(i);
    int x_separate_index = -1;
    int y_separate_index = -1;
    for(int j_x = x_fragment_number_; j_x > 0 ; --j_x) {
      if(point.x >= x_low_limitation_[j_x]) {
        x_separate_index = j_x;
        break;
      }
    } for(int j_y = y_fragment_number_; j_y > 0; --j_y) {
      if(point.y >= y_low_limitation_[j_y]) {
        y_separate_index = j_y;
        break;
      }
    } assert(x_separate_index == -1);
    assert(y_separate_index == -1);
    fragment_points_[x_separate_index][y_separate_index].push_back(point); // index start from 1
  } for(int i = 1; i <= x_fragment_number_; ++i) {
    for(int j = 1; j <= y_fragment_number_; ++j) {
      SinglePartExtract(fragment_points_[i][j]);
    }
  }
}

void GroundFeature::SinglePartExtract(pcl::PointCloud<FM::PointTypeGround> &single_points) {
  pcl::PointCloud<PointTypeGround>& process_cloud = single_points;
  std::sort(process_cloud.points.begin() , process_cloud.points.end() , [](const PointTypeGround a, const PointTypeGround b){
    return a.z < b.z;
  }); pcl::PointCloud<PointTypeGround>::iterator it = process_cloud.points.begin();
  size_t si = process_cloud.points.size();
  double calculate_sensor_height = -1.5 * sensor_height_;
  for(register size_t i = 0 ; i < si ; ++i) {
    if(process_cloud.points[i].z < -1.5 ) {
      ++it;
    } else {
      break;
    }
  } process_cloud.points.erase(process_cloud.points.begin() , it);
  // extract init ground seeds
  ExtractInitialSeeds(process_cloud);

  g_ground_pc_ = g_seeds_pc_;

  // ground plane fitter main
  for(int i = 0 ; i < num_ground_iterator_ ; ++i) {
    EstimatePlane();
    g_ground_pc_->clear();
    g_non_ground_pc_->clear();

    // point cloud to matrix
    Eigen::MatrixXf points(single_points.points.size(), 3);
    int j = 0;
    for(auto p: single_points.points) {
      points.row(j++) << p.x , p.y , p.z;
    }
    // ground plane model
    Eigen::VectorXf result = points * normal_;
    // threshold filter
    size_t row_si = points.rows();
    for(size_t r = 0 ; r < row_si ; ++r) {
      if(result[r] < th_dist_d_) {
        single_points.points.at(r).intensity = 0;
        g_ground_pc_->points.push_back(single_points.points[r]);
      } else {
        single_points.points.at(r).intensity = 1;
        g_non_ground_pc_->points.push_back(single_points.points[r]);
      }
    }
  }
}

void GroundFeature::ExtractInitialSeeds(pcl::PointCloud<FM::PointTypeGround> &process_cloud) {
  // LPR is the mean of low point representative
  double sum = 0;
  double count = 0;
  // calculate the mean height value
  size_t si = process_cloud.points.size();
  for(size_t i = 0 ; i < si && count < num_lpr_; ++i) {
    sum += process_cloud.points[i].z;
    ++count;
  } double lpr_height = count ? sum / count : 0;
  g_seeds_pc_->clear();
  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for(size_t i = 0 ; i < si ; ++i) {
    if(process_cloud.points[i].z < lpr_height + ground_seeds_threshold_) {
      g_seeds_pc_->points.push_back(process_cloud.points[i]);
    }
  }
  // return seeds points
}

void GroundFeature::EstimatePlane() {
  // create covariance matrix in signal pass
  // TODO: compare the efficiency
  Eigen::Matrix3f cov;
  Eigen::Vector4f pc_mean;
  pcl::computeMeanAndCovarianceMatrix(*g_ground_pc_, cov, pc_mean);
  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));
  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean.head<3>();
  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose()*seeds_mean)(0,0);
  // set distance threhold to `ground_distance_threshold - d`
  th_dist_d_ = ground_distance_threshold_ - d_;

  // return the equation parameters
}

pcl::PointCloud<PointTypeGround> GroundFeature::groundWithSAC(pcl::PointCloud<PointTypeGround> &raw_points,
                                                              Eigen::Vector3d &ground_normal) {
  pcl::SACSegmentation<PointTypeGround> segmentation;
  pcl::PointCloud<PointTypeGround> cloud_projected;
  pcl::PointCloud<PointTypeGround>::Ptr cloud_filtered(new pcl::PointCloud<PointTypeGround>);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  *cloud_filtered = raw_points;
  segmentation.setOptimizeCoefficients (true);
  segmentation.setModelType (pcl::SACMODEL_PLANE);
  segmentation.setMethodType (pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold (0.2);
  segmentation.setInputCloud (cloud_filtered);
  segmentation.segment (*inliers, *coefficients);
  double a_ = coefficients->values[0], b_ = coefficients->values[1], c_ = coefficients->values[2], d_ = coefficients->values[3];
//  printf("plane: %f x + %f y + %f z = %f.",a_, b_, c_, d_);
  //double distance_zero = std::fabs(d_) / sqrt(a_ * a_ + b_ * b_ + c_ * c_);
  double distance_zero = std::fabs(d_) / c_;
//  printf("distance to zero:{%f}\n",distance_zero);
  ground_normal = Eigen::Vector3d(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
  int si = inliers->indices.size();

  for (int j = 0; j < si ; ++j) {
    cloud_projected.push_back(cloud_filtered->points[inliers->indices[j]]);
  }
  raw_points = cloud_projected;
  return cloud_projected;
}

}