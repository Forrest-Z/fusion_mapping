//
// Created by linsin on 13/02/2020.
//

#include "fusion_mapping/core/sensor_data/gnss_data.h"
#include "fusion_mapping/core/util/util.hpp"
#include <cstdio>


//静态成员变量必须在类外初始化
bool FM::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian FM::GNSSData::geo_converter;

namespace FM {

void GNSSData::InitOriginPosition() {
  geo_converter.Reset(latitude, longitude, altitude);
  origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
  if (!origin_position_inited) {
    printf(FONT_COLOR_RED "GeoConverter has not set origin position" COLOR_NONE);
  }
  geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}
}

