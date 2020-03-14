//
// Created by linsin on 12/03/2020.
//

#include <ros/ros.h>
#include "glog/logging.h"

#include <fusion_mapping/saveMap.h>
#include "fusion_mapping/core/global_defination/global_defination.h"
#include "fusion_mapping/core/mapping/viewer/viewer_flow.h"

using namespace FM;
using namespace fusion_mapping;

std::shared_ptr<ViewerFlow> _viewer_flow_ptr;
bool _need_save_map = false;

bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
  _need_save_map = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "viewer_node");
  ros::NodeHandle nh;

  std::string cloud_topic;
  nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
  std::shared_ptr<ViewerFlow> _viewer_flow_ptr = std::make_shared<ViewerFlow>(nh, cloud_topic);

  ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    _viewer_flow_ptr->Run();
    if (_need_save_map) {
      _need_save_map = false;
      _viewer_flow_ptr->SaveMap();
    }

    rate.sleep();
  }

  return 0;
}