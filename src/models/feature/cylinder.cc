//
// Created by linsin on 18/04/2020.
//

#include "fusion_mapping/core/models/feature/cylinder.h"

namespace FM{
CylinderCoefficient CylinderRelated::CylinderFitting(pcl::PointCloud<pcl::PointXYZ> &cloud) {
  Eigen::MatrixXd params(7 , 1);
  params << 0, 0, 0, 0, 0, 0, 0;
  Eigen::MatrixXd X(7, 1);
  X << 8, 7, 6, 5, 1, 2, 3;
  int times = 0;
  while((fabs(X(3, 0)) + fabs(X(4, 0)) + fabs(X(5 , 0))) > 0.001 || fabs(X(6, 0)) > 0.01 || (!times)) {
    params += X;
    std::cout << "param = " << params << std::endl;
    float x0 = params(0, 0);	float y0 = params(1, 0);	float z0 = params(2, 0);
    float a0 = params(3, 0);	float b0 = params(4, 0);	float c0 = params(5, 0);
    float r0 = params(6, 0);
    float f0;
    int size = cloud.points.size();		//point size, M size
    Eigen::MatrixXd m(size, 7), mT(7, size), B(size, 1);
    for (int i = 0; i < size; i++) {
      float x = cloud.points[i].x;
      float y = cloud.points[i].y;
      float z = cloud.points[i].z;

      f0 = pow(x - x0, 2) + pow(y - y0, 2) + pow(z - z0, 2) - pow((a0*(x - x0) + b0 * (y - y0) + c0 * (z - z0)), 2) - r0 * r0;
      B(i, 0) = -f0;
      m(i, 0) = (a0 * (a0*(x - x0) + b0 * (y - y0) + c0 * (z - z0)) - (x - x0)) * 2;
      m(i, 1) = (b0 * (a0*(x - x0) + b0 * (y - y0) + c0 * (z - z0)) - (y - y0)) * 2;
      m(i, 2) = (c0 * (a0*(x - x0) + b0 * (y - y0) + c0 * (z - z0)) - (z - z0)) * 2;
      m(i, 3) = -(a0*(x - x0) + b0 * (y - y0) + c0 * (z - z0))*(x - x0) * 2;
      m(i, 4) = -(a0*(x - x0) + b0 * (y - y0) + c0 * (z - z0))*(y - y0) * 2;
      m(i, 5) = -(a0*(x - x0) + b0 * (y - y0) + c0 * (z - z0))*(z - z0) * 2;
      m(i, 6) = -2*r0;
    }
    X = m.colPivHouseholderQr().solve(B);			//改正数
    std::cout << "\nX=" << X << std::endl;

    ////  V=Ax-B,  x=(ATA)-1AT*B
    //mT = m.transpose();
    //MatrixXd mTm(7, 7), mTm_inv(7, 7);
    //mTm = mT * m;
    //mTm_inv = mTm.inverse();
    //X = mTm_inv * mT*B;
    //std::cout << "\nX=" << X << std::endl;
    ++times;
  }

  CylinderCoefficient final_params;
  final_params.x = params(0, 0);
  final_params.y = params(1, 0);
  final_params.z = params(2, 0);
  final_params.l = params(3, 0);
  final_params.m = params(4, 0);
  final_params.n = params(5, 0);
  final_params.r = params(6, 0);

  return final_params;

}
}