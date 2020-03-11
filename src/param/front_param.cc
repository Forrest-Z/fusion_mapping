//
// Created by linsin on 11/03/2020.
//
#include "fusion_mapping/core/param/front_param.h"

namespace FM{
LidarConfig FrontParam::lidar_config_;
GNSSConfig FrontParam::gnss_config_;
IMUConfig FrontParam::imu_config_;
std::shared_ptr<FrontParam> FrontParam::front_param_ = nullptr;
bool FrontParam::LoadLidarConfig() {
  if(front_param_ == nullptr) {
    front_param_ = std::shared_ptr<FrontParam>(new FrontParam);
  } YAML::Node node;
  try {
    node = YAML::LoadFile(lidar_config_file);
  } catch (YAML::BadFile &e) {
    std::cout << e.msg << std::endl;
    return 0;
  }

  front_param_->lidar_config_.topic_name = node["topic-name"][0].as<std::string>();
  front_param_->lidar_config_.ring_number = node["ring-number"][0].as<uint16_t >();
  front_param_->lidar_config_.vertical_resolution = node["vertical-resolution"][0].as<double>();
  front_param_->lidar_config_.horizontal_resolution = node["horizontal-resolution"][0].as<double>();
  front_param_->lidar_config_.closed_resolution = node["closed-resolution"][0].as<double>();
  front_param_->lidar_config_.sensor_height = node["sensor-height"][0].as<double>();
  front_param_->lidar_config_.ground_seeds_threshold = node["ground-seeds-threshold"][0].as<double>();
  front_param_->lidar_config_.ground_distance_threshold = node["ground-distance-threshold"][0].as<double>();
  front_param_->lidar_config_.num_ground_iterator = node["num-ground-iterator"][0].as<int>();
  front_param_->lidar_config_.num_lpr = node["num-lpr"][0].as<int>();
}

bool FrontParam::LoadGNSSConfig() {
  if (front_param_ == nullptr) {
    front_param_ = std::shared_ptr<FrontParam>(new FrontParam);
  }
  YAML::Node node;
  try {
    node = YAML::LoadFile(gnss_config_file);
  } catch (YAML::BadFile &e) {
    std::cout << e.msg << std::endl;
    return 0;
  }

  front_param_->gnss_config_.topic_name = node["topic-name"][0].as<std::string>();
}

bool FrontParam::LoadIMUConfig() {
  if (front_param_ == nullptr) {
    front_param_ = std::shared_ptr<FrontParam>(new FrontParam);
  }
  YAML::Node node;
  try {
    node = YAML::LoadFile(imu_config_file);
  } catch (YAML::BadFile &e) {
    std::cout << e.msg << std::endl;
    return 0;
  }

  front_param_->imu_config_.topic_name = node["topic-name"][0].as<std::string>();
}

}
