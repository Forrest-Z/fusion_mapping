//
// Created by linsin on 12/03/2020.
//

#include "fusion_mapping/core/models/camera/rgbd/rgbd_base.h"
#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/nonfree/nonfree.hpp> // use this if you want to use SIFT or SURF
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

int main(int argc, char** argv) {
  cv::Mat rgb1 = cv::imread("./data/rgb1.png");
  cv::Mat rgb2 = cv::imread("./data/rgb2.png");
  cv::Mat depth1 = cv::imread("./data/depth1.png", -1);
  cv::Mat depth2 = cv::imread("./data/depth2.png", -1);

  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> descriptor;

  // 如果使用 sift, surf ，之前要初始化nonfree模块
  // cv::initModule_nonfree();
  // _detector = cv::FeatureDetector::create( "SIFT" );
  // _descriptor = cv::DescriptorExtractor::create( "SIFT" );

  detector = cv::FeatureDetector::create("ORB");
  descriptor = cv::DescriptorExtractor::create("ORB");

  std::vector<cv::KeyPoint> kp1, kp2;
  detector->detect(rgb1, kp1);
  detector->detect(rgb2, kp2);

  std::cout << "rgb1 key point size: " << kp1.size() << "rab2 key point size: " << kp2.size() << std::endl;

  // 可视化， 显示关键点
  cv::Mat imgShow;
  cv::drawKeypoints( rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  cv::imshow( "keypoints", imgShow );
  cv::imwrite( "./data/keypoints.png", imgShow );
  cv::waitKey(0); //暂停等待一个按键

  // 计算描述子
  cv::Mat desp1, desp2;
  descriptor->compute(rgb1, kp1, desp1);
  descriptor->compute(rgb2, kp2, desp2);

  // 匹配描述子
  std::vector<cv::DMatch> matches;
  cv::BFMatcher matcher;
  matcher.match(desp1, desp2, matches);

  //显示匹配的特征
  cv::Mat imgMatches;
  cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
  cv::imshow( "matches", imgMatches );
  cv::imwrite( "./data/matches.png", imgMatches );
  cv::waitKey( 0 );

  //filter matches, drop matches that are too long
  // 这里使用的准则是去掉大于四倍最小距离的匹配

  std::vector<cv::DMatch> goodMatches;
  double minDis = 99999;
  for(int i = 0 ; i < matches.size() ; ++i) {
    if(matches.at(i).distance < minDis) {
      minDis = matches.at(i).distance;
    }
  }

  for(int i = 0 ; i < matches.size() ; ++i) {
    if(matches.at(i).distance < 10 * minDis) {
      goodMatches.push_back(matches.at(i));
    }
  }

  // 显示 good matches
  cout<<"good matches = "<<goodMatches.size()<<endl;
  cv::drawMatches( rgb1, kp1, rgb2, kp2, goodMatches, imgMatches );
  cv::imshow( "good matches", imgMatches );
  cv::imwrite( "./data/good_matches.png", imgMatches );
  cv::waitKey(0);

  FM::RGBDBase rgbd_base;
  // 第一个帧的三维点
  vector<cv::Point3f> pts_obj;
  // 第二个帧的图像点
  vector< cv::Point2f > pts_img;
  for (size_t i = 0; i < goodMatches.size(); ++i) {
    // query 是第一个, train 是第二个
    cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
    // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
    ushort d = depth1.ptr<ushort>( int(p.y) )[ int(p.x) ];
    if (d == 0)
      continue;
    pts_img.push_back( cv::Point2f( kp2[goodMatches[i].trainIdx].pt ) );

    // 将(u,v,d)转成(x,y,z)
    cv::Point3f pt ( p.x, p.y, d );
    cv::Point3f pd = point2dTo3d( pt, C );
    pts_obj.push_back( pd );
  }

  double camera_matrix_data[3][3] = {
      {rgbd_base.camera_fx_, 0, rgbd_base.camera_cx_},
      {0, rgbd_base.camera_fy_, rgbd_base.camera_cy_},
      {0, 0, 1}
  };

  // 构建相机矩阵
  cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
  cv::Mat rvec, tvec, inliers;
  // 求解pnp
  cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );

  cout<<"inliers: "<<inliers.rows<<endl;
  cout<<"R="<<rvec<<endl;
  cout<<"t="<<tvec<<endl;

  // 画出inliers匹配
  vector< cv::DMatch > matchesShow;
  for (size_t i=0; i<inliers.rows; i++)
  {
    matchesShow.push_back( goodMatches[inliers.ptr<int>(i)[0]] );
  }
  cv::drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
  cv::imshow( "inlier matches", imgMatches );
  cv::imwrite( "./data/inliers.png", imgMatches );
  cv::waitKey( 0 );

}

