//
// Created by linsin on 11/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PARAM_FRONT_PARAM_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PARAM_FRONT_PARAM_H_

#include <yaml-cpp/yaml.h>
#include <map>
#include <iostream>
#include <fstream>
#include "fusion_mapping/core/global_defination/global_defination.h"

namespace FM{
const std::string lidar_config_file = WORK_SPACE_PATH+std::string("/config/front_end/lidar_config.yaml");
struct LidarConfig{
  std::string topic_name;
  uint16_t ring_number;
  double vertical_resolution;
  double horizontal_resolution;
  double closed_resolution;
  double sensor_height;
  double ground_seeds_threshold; // 用于选取种子点的阈值，当点云内的点的高度小于LPR的高度加上此阈值时，我们将该点加入种子点集
  double ground_distance_threshold; // 平面距离阈值，我们会计算点云中每一个点到我们拟合的平面的正交投影的距离，而这个平面距离阈值，就是用来判定点是否属于地面
  double num_ground_iterator; // 优化次数
  double num_lpr; // 选取的lpr个数
};

struct FrontParam{
  static LidarConfig lidar_config_;
  static bool LoadLidarConfig();
 private:
  static std::shared_ptr<FM::FrontParam> front_param_;

};

}


#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_PARAM_FRONT_PARAM_H_
