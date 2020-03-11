//
// Created by linsin on 13/02/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_SENSOR_DATA_GNSS_DATA_H_
#define FUSION_MAPPING_INCLUDE_SENSOR_DATA_GNSS_DATA_H_

#include <vector>
#include <string>

#include "Geocentric/LocalCartesian.hpp"

using std::vector;
using std::string;

namespace FM{
class GNSSData {
 public:
  double time = 0.0;
  double longitude = 0.0;
  double latitude = 0.0;
  double altitude = 0.0;
  double local_E = 0.0;
  double local_N = 0.0;
  double local_U = 0.0;
  int status = 0;
  int service = 0;

 private:
  static GeographicLib::LocalCartesian geo_converter;
  static bool origin_position_inited;

 public:
  void InitOriginPosition();
  void UpdateXYZ();
};
}

#endif //FUSION_MAPPING_INCLUDE_SENSOR_DATA_GNSS_DATA_H_
