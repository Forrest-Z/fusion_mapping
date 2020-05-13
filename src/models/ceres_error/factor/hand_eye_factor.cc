//
// Created by linsin on 28/04/2020.
//

#include "fusion_mapping/core/models/ceres_error/factor/hand_eye_factor.h"
namespace FM{
HandEyeFactor::HandEyeFactor(const FM::PoseParameterization &lidar_odom, const FM::PoseParameterization &gnss_odom) {
  setMeasurementsAndGNSS(lidar_odom, gnss_odom);
} bool HandEyeFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  // get pose
  const okvis::kinematics::Transformation T_gl(
      Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]));
  // this will not be changed
  const Eigen::Matrix3d& R_gl = T_gl.C();
  const Eigen::Vector3d t_gl = T_gl.r();

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = R_gl;
  T.block<3, 1>(0, 3) = t_gl;

  Eigen::Matrix4d gnss = gnss_odom_.estimate().T();
  Eigen::Matrix4d lidar = lidar_odom_.estimate().T();

  Eigen::Matrix4d res = gnss * T - T * lidar;
  Eigen::Quaterniond rotation_qua(res.block<3,3>(0,0));
  Eigen::Vector3d angle = rotation_qua.vec(); // z y x

  residuals[0] = res(0, 3);
  residuals[1] = res(1, 3);
  residuals[2] = res(2, 3);
  residuals[3] = angle[2];
  residuals[4] = angle[1];
  residuals[5] = angle[0];

  if(jacobians) {
   if(jacobians[0]) {
     // TODO: checkout the jacobian ...
     Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > jacobian_pose(jacobians[0]);
     jacobian_pose.setZero();
     Eigen::Matrix4d J1 = gnss;
     Eigen::Matrix4d J2 = T;
     jacobian_pose = J1 - J2;
   }
  }
  return true;

}
}