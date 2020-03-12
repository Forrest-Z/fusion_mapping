//
// Created by linsin on 12/03/2020.
//

/*
 * Function:
 *  1. extract keyframe
 *  2. GNSS and keyframe constraint, loop closure constraint
 * Input:
 *  1. front end odom
 *  2. GNSS odom
 *  3. loop closure
 * Output:
 *  1. pose after optimization
 */
#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_BACK_END_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_BACK_END_H_
#include <string>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "fusion_mapping/core/sensor_data/cloud_data.h"
#include "fusion_mapping/core/sensor_data/pose_data.h"
#include "fusion_mapping/core/sensor_data/key_frame.h"
#include "fusion_mapping/core/global_defination/global_defination.h"
#include "fusion_mapping/core/tools/file_manager.h"

namespace FM {
class BackEnd {
 public:
  BackEnd();

  bool Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose);

  void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);
  bool HasNewKeyFrame();
  bool HasNewOptimized();
  void GetLatestKeyFrame(KeyFrame& key_frame);

 private:
  bool InitWithConfig();
  bool InitParam(const YAML::Node& config_node);
  bool InitDataPath(const YAML::Node& config_node);

  void ResetParam();
  bool SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose);
  bool MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom);
  bool MaybeOptimized();

 private:
  std::string key_frames_path_ = "";
  std::string trajectory_path_ = "";

  std::ofstream ground_truth_ofs_;
  std::ofstream laser_odom_ofs_;

  float key_frame_distance_ = 2.0;
  int optimize_step_with_none_ = 100;
  int optimize_step_with_gnss_ = 100;
  int optimize_step_with_loop_ = 10;

  bool has_new_key_frame_ = false;
  bool has_new_optimized_ = false;

  Eigen::Matrix4f last_key_pose_ = Eigen::Matrix4f::Identity();
  KeyFrame latest_key_frame_;
  std::deque<KeyFrame> key_frames_deque_;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MAPPING_BACK_END_H_
