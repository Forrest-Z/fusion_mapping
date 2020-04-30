//
// Created by linsin on 12/03/2020.
//

#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "glog/logging.h"
#include "fusion_mapping/core/global_defination/global_defination.h"
#include "fusion_mapping/core/models/ceres_error/parameterization/pose_parameterization.h"
#include "fusion_mapping/core/models/ceres_error/local_param/pose_local_parameterization.h"
#include "fusion_mapping/core/models/ceres_error/factor/hand_eye_factor.h"

using namespace FM;

int main(int argc, char *argv[]) {

  ceres::Problem problem;
  okvis::kinematics::Transformation TWC;
  TWC.setRandom();
  PoseParameterization poseTWC(TWC, 0);
  okvis::kinematics::Transformation odom;
  odom.setRandom();
  PoseParameterization lidar(TWC, 1);
  PoseParameterization gnss(TWC, 2);
  ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
  problem.AddParameterBlock(poseTWC.parameters(), poseTWC.dimension(), local_parameterization);

  HandEyeFactor* factor = new HandEyeFactor(lidar, gnss);
  //problem.AddResidualBlock(factor, NULL, cylinderInitialModel.parameters(), poseTWC.parameters());
  problem.AddResidualBlock(factor, NULL, poseTWC.parameters());

  ceres::Solver::Options options;

  options.linear_solver_type = ceres::DENSE_QR;  // ceres::DENSE_QR
  //options.num_threads = 2;
  options.trust_region_strategy_type = ceres::DOGLEG;  // ceres::LEVENBERG_MARQUARDT
  options.max_num_iterations = 500;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = 1e-16;
  //options.max_num_consecutive_invalid_steps = 100;

  ceres::Solver::Summary summary;
  //std::cout << "test test5" << std::endl;
  ceres::Solve(options, &problem, &summary);
  //std::cout << "test test6" << std::endl;
  std::cout << summary.FullReport() << std::endl;


  return 0;
}