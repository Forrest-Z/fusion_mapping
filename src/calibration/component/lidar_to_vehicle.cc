//
// Created by linsin on 23/04/2020.
//

#include "fusion_mapping/core/calibration/component/lidar_to_vehicle.h"

namespace FM{
Eigen::Matrix3d LidarToVehicle::calibrationRollPitchWithNormal(Eigen::Vector3d &ground_normal,
                                                               Eigen::Vector3d &std_normal) {
  ground_normal.normalize();
  std_normal.normalize();
  double angle = acos(ground_normal.dot(std_normal));
  Eigen::Vector3d p_rotate = ground_normal.cross(std_normal);
  p_rotate.normalize();
  Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
  rotationMatrix(0, 0)=cos(angle)+ p_rotate[0] * p_rotate[0] * (1 - cos(angle));
  rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));//这里跟公式比多了一个括号，但是看实验结果它是对的。
  rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
  rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
  rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
  rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
  rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
  rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
  rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

  return rotationMatrix;
}
}

