//
// Created by linsin on 12/03/2020.
//

#include <ros/ros.h>
#include <fusion_mapping/optimizeMap.h>
#include "glog/logging.h"
#include "fusion_mapping/core/global_defination/global_defination.h"
#include "fusion_mapping/core/mapping/back_end/back_end_flow.h"


using namespace FM;
using namespace fusion_mapping;

std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request &request, optimizeMap::Response &response) {
  _need_optimize_map = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "back_end_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("optimize_map", optimize_map_callback);
  _back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    _back_end_flow_ptr->Run();

    if (_need_optimize_map) {
      _back_end_flow_ptr->ForceOptimize();
      _need_optimize_map = false;
    }

    rate.sleep();
  }

  return 0;
}