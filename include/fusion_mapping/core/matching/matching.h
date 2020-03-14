//
// Created by linsin on 14/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MATCHING_MATCHING_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MATCHING_MATCHING_H_
#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "fusion_mapping/core/sensor_data/cloud_data.h"
#include "fusion_mapping/core/sensor_data/pose_data.h"
#include "fusion_mapping/core/models/registration/registration_interface.h"
#include "fusion_mapping/core/models/cloud_filter/cloud_filter_interface.h"
#include "fusion_mapping/core/models/cloud_filter/box_filter.h"

namespace FM {
class Matching {
 public:
  Matching();

  bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
  bool SetGNSSPose(const Eigen::Matrix4f& init_pose);

  void GetGlobalMap(CloudData::CLOUD_PTR& global_map);
  CloudData::CLOUD_PTR& GetLocalMap();
  CloudData::CLOUD_PTR& GetCurrentScan();
  bool HasInited();
  bool HasNewGlobalMap();
  bool HasNewLocalMap();

 private:
  bool InitWithConfig();
  bool InitDataPath(const YAML::Node& config_node);
  bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
  bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
  bool InitBoxFilter(const YAML::Node& config_node);

  bool SetInitPose(const Eigen::Matrix4f& init_pose);
  bool InitGlobalMap();
  bool ResetLocalMap(float x, float y, float z);

 private:
  std::string map_path_ = "";

  std::shared_ptr<BoxFilter> box_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
  std::shared_ptr<RegistrationInterface> registration_ptr_;

  CloudData::CLOUD_PTR local_map_ptr_;
  CloudData::CLOUD_PTR global_map_ptr_;
  CloudData::CLOUD_PTR current_scan_ptr_;
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();
  bool has_inited_ = false;
  bool has_new_global_map_ = false;
  bool has_new_local_map_ = false;
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MATCHING_MATCHING_H_
