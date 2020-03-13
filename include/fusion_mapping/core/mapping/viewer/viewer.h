//
// Created by linsin on 12/03/2020.
//

/*
 * Function:
 *  1. product map according optimized pose
 *  2. visualize map and current point cloud
 * Input:
 *  1. current point cloud
 *  2. current pose
 *  3. optimized history pose
 * Output:
 *  1. current point cloud according optimized pose
 *  2. current pose according optimized pose
 *  3. local map
 *  4. global map
 */

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_VIEWER_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_VIEWER_H_
#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "fusion_mapping/core/sensor_data/cloud_data.h"
#include "fusion_mapping/core/sensor_data/key_frame.h"
#include "fusion_mapping/core/sensor_data/pose_data.h"
#include "fusion_mapping/core/models/cloud_filter/voxel_filter.h"
#include "fusion_mapping/core/global_defination/global_defination.h"

namespace FM {
class Viewer {
 public:
  Viewer();

  bool UpdateWithOptimizedKeyFrames(std::deque<KeyFrame>& optimized_key_frames);
  bool UpdateWithNewKeyFrame(std::deque<KeyFrame>& new_key_frames,
                               PoseData transformed_data,
                               CloudData cloud_data);

  bool SaveMap();
  Eigen::Matrix4f& GetCurrentPose();
  CloudData::CLOUD_PTR& GetCurrentScan();
  bool GetLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
  bool GetGlobalMap(CloudData::CLOUD_PTR& local_map_ptr);
  bool HasNewLocalMap();
  bool HasNewGlobalMap();

 private:
  bool InitWithConfig();
  bool InitParam(const YAML::Node& config_node);
  bool InitDataPath(const YAML::Node& config_node);
  bool InitFilter(std::string filter_user,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr,
                  const YAML::Node& config_node);

  void ResetParam();
  bool OptimizeKeyFrames();
  bool JointGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
  bool JointLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
  bool JointCloudMap(const std::deque<KeyFrame>& key_frames,
                     CloudData::CLOUD_PTR& map_cloud_ptr);

 private:
  std::string data_path_ = "";
  int local_frame_num_ = 20;

  std::string key_frames_path_ = "";
  std::string map_path_ = "";

  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

  Eigen::Matrix4f pose_to_optimize_ = Eigen::Matrix4f::Identity();
  PoseData optimized_odom_;
  CloudData optimized_cloud_;
  std::deque<KeyFrame> optimized_key_frames_;
  std::deque<KeyFrame> all_key_frames_;

  bool has_new_global_map_ = false;
  bool has_new_local_map_ = false;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_VIEWER_H_
