//
// Created by linsin on 14/03/2020.
//

#include <ros/ros.h>
#include "glog/logging.h"

#include "fusion_mapping/core/global_defination/global_defination.h"
#include "fusion_mapping/core/matching/matching_flow.h"

using namespace FM;

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "matching_node");
  ros::NodeHandle nh;

  std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(nh);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    matching_flow_ptr->Run();

    rate.sleep();
  }

  return 0;
}