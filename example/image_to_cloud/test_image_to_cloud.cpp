//
// Created by linsin on 12/03/2020.
//

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
  double min_d = 999999999;
  pcl::PointXYZ min_p;

  //生成圆柱点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (float z(-10); z <= 10; z += 0.5) {
    for (float angle(0.0); angle <= 360.0; angle += 5.0) {
      pcl::PointXYZ basic_point;
      basic_point.x = 3.5*cos(angle / 180 * M_PI);
      basic_point.y = 3.5*sin(angle / 180 * M_PI);
      basic_point.z = z;
      cloud->points.push_back(basic_point);
    }
  }
  Eigen::Vector3d normal(0, 0 , 1);
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d x_matrix =Eigen::AngleAxisd((30.0) * M_PI / 189.0, Eigen::Vector3d(1,0,0)).toRotationMatrix();
  Eigen::Matrix3d y_matrix =Eigen::AngleAxisd((30.0) * M_PI / 180.0, Eigen::Vector3d(0,1,0)).toRotationMatrix();
  Eigen::Matrix3d z_matrix = Eigen::AngleAxisd((0.0) * M_PI / 180.0, Eigen::Vector3d(0,0,1)).toRotationMatrix();
  trans.block<3,3>(0,0) = z_matrix * y_matrix * x_matrix;
  trans.block<3, 1>(0, 3) = Eigen::Vector3d(10, 200, 0);
  pcl::transformPointCloud(*cloud, *cloud, trans);
  normal = trans.block<3, 3>(0 , 0) * normal + trans.block<3 , 1>(0 , 3);

//  for (float z(-10); z <= 10; z += 0.5) {
//    for (float angle(0.0); angle <= 360.0; angle += 5.0) {
//      pcl::PointXYZ basic_point;
//      basic_point.x = 3.5*cos(angle / 180 * M_PI);
//      basic_point.y = 3.5*sin(angle / 180 * M_PI);
//      basic_point.z = z;
//      cloud->points.push_back(basic_point);
//    }
//  }
//  cloud->points.push_back({0 , 0 , 0});

  cloud->height = 1;
  cloud->width = cloud->size();
  cloud->is_dense = false;
  int si = cloud->size();
  for(int i = 0 ; i < si ; ++i) {
    double dis = sqrt(cloud->points.at(i).x * cloud->points.at(i).x +
        cloud->points.at(i).y * cloud->points.at(i).y +
    cloud->points.at(i).z * cloud->points.at(i).z);
    if(dis < min_d) {
      min_p = cloud->points.at(i);
      min_d = dis;
    }
  }
  std::cout << "min point is " << min_p.x << " " << min_p.y << " " << min_p.z << std::endl;
  std::cout << "normal is " << normal(0, 0) << " " << normal(1, 0) << " " << normal(2, 0) << std::endl;
  std::cout << "r is " << 3.5 << std::endl;
  pcl::io::savePCDFileASCII("/home/linsin/fuck/cylinder.pcd", *cloud);
}

