//
// Created by linsin on 15/02/2020.
//
#include <ros/ros.h>
#include "glog/logging.h"

#include <fusion_mapping/saveMap.h>
#include "fusion_mapping/core/global_defination/global_defination.h"
#include "fusion_mapping/core/front_end/front_end_flow.h"

using namespace FM;
using namespace fusion_mapping;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
  response.succeed = _front_end_flow_ptr->SaveMap();
  _front_end_flow_ptr->PublishGlobalMap();
  return response.succeed;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "front_end_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
  _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    _front_end_flow_ptr->Run();

    rate.sleep();
  }

  return 0;
}
