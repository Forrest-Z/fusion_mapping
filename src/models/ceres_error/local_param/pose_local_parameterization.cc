//
// Created by linsin on 30/04/2020.
//
#include "models/ceres_error/local_param/pose_local_parameterization.h"

namespace FM{
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
  Eigen::Matrix<double , 6, 1> delta_;
  delta_.setZero();
  delta_[0] = delta[0];
  delta_[1] = delta[1];
  delta_[3] = delta[3];
  delta_[4] = delta[4];
  okvis::kinematics::Transformation T(
      Eigen::Vector3d(x[0], x[1], x[2]),
      Eigen::Quaterniond(x[6], x[3], x[4], x[5]));
  T.oplus(delta_);
  // copy back
  x_plus_delta[0] = T.r()[0];
  x_plus_delta[1] = T.r()[1];
  x_plus_delta[2] = T.r()[2];
  x_plus_delta[3] = T.q().coeffs()[0];
  x_plus_delta[4] = T.q().coeffs()[1];
  x_plus_delta[5] = T.q().coeffs()[2];
  x_plus_delta[6] = T.q().coeffs()[3];
  return true;

}

bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double , 7 , 6>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();
  return true;
}
}

