#ifndef RECONSTRUCTION3DSOVIN_H
#define RECONSTRUCTION3DSOVIN_H

/**
* @author Léo Baudouin\n
* @em baudouin.leo@gmail.com
*/

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv/cv.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <stdio.h>

#include "tools.h"

#include "../test/MAlgo5Points.h"
#include "../test/MathUtils.h"
#include "../test/matrix.h"
#include "../test/Matrix33.h"
#include "../test/MCamera.h"


using namespace local_vis_eric;
using namespace ns_lasmea;


/** @class Reconstruction3D
* @short Define several method to reconstruct 3D view from multiple view geometry
*/
class Reconstruction3DSovin
{
public:
  Reconstruction3DSovin();

  void firstBuild(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat K, cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &pts3D);
  void findExtrinsicPositions(cv::Mat E, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec);
  void findBestExtrinsicPosition(std::vector<cv::Mat> Rvec, std::vector<cv::Mat> tvec, cv::Mat K, std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &pts3D);

  void triangulate(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, std::vector<cv::Point3d> &pts3D, cv::Mat R0, cv::Mat t0, cv::Mat R1, cv::Mat t1, cv::Mat K);
  void computeExtrinsicParameters(std::vector<cv::Point2d> pts2D, std::vector<cv::Point3d> pts3D, cv::Mat K, cv::Mat &R, cv::Mat &t);


  cv::Point2d projectPoint(cv::Mat R, cv::Mat t, cv::Mat K, cv::Point3d pt3D);
  std::vector<cv::Point2d> projectPoints(cv::Mat R, cv::Mat t, cv::Mat K, std::vector<cv::Point3d> pts3D);

  cv::Mat getRetropjImage(cv::Mat img, std::vector<cv::Point2d> pts_input, std::vector<cv::Point2d> pts_reproj, std::vector<bool> good_matching = std::vector<bool>());

  bool isForward(cv::Mat R, cv::Mat t, cv::Point3d pt);

  cv::Mat toCvMat(Matrix33 m)
  {
    cv::Mat M = cv::Mat::zeros(3,3,CV_64F);
    M.at<double>(0,0) = m.Get00();
    M.at<double>(0,1) = m.Get01();
    M.at<double>(0,2) = m.Get02();
    M.at<double>(1,0) = m.Get10();
    M.at<double>(1,1) = m.Get11();
    M.at<double>(1,2) = m.Get12();
    M.at<double>(2,0) = m.Get20();
    M.at<double>(2,1) = m.Get21();
    M.at<double>(2,2) = m.Get22();
    return M;
  }

private:
  MAlgo5Points MA5P;
};

#endif //RECONSTRUCTION3DSOVIN_H
