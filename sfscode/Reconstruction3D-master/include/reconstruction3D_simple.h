#ifndef RECONSTRUCTION3DSIMPLE_H
#define RECONSTRUCTION3DSIMPLE_H

/**
* @author Léo Baudouin\n
* @em baudouin.leo@gmail.com
*/

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv/cv.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <stdio.h>

#include "timer.h"
#include "tools.h"
//#include "color.h"

#include "../include/datamanager.h"

//#include "../test/MAlgo5Points.h"

#include "Matcher/matcher.h"

#include "../include/imageprocessing.h"
#include "../include/fileprocessing.h"
#include "../include/gnuplot.h"

#include <omp.h>

#include "../include/loopclosure.h"

//#include "../include/sba_ceres.h"

class Pose
{
public:
  Pose(cv::Mat R, cv::Mat t) : R_(R) , t_(t) {}
  cv::Mat getR() {return R_;}
  cv::Mat getT() {return t_;}
private:
  cv::Mat R_,t_;
};

/** @class Reconstruction3D
* @short Define several method to reconstruct 3D view from multiple view geometry
*/
class Reconstruction3DSimple
{
public:
  Reconstruction3DSimple();

  int reconstruct(std::vector<std::string> imageList, cv::Mat K, cv::Mat dist, Matcher *matcher, DataManager *dm, int step = 16);
  //bool addImage(cv::Mat &image1, cv::Mat &image2, std::vector<PointData> &lastPoint);

  void firstBuild(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat K, cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &pts3D);
  void findExtrinsicPositions(cv::Mat E, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec);
  void findBestExtrinsicPosition(std::vector<cv::Mat> Rvec, std::vector<cv::Mat> tvec, cv::Mat K, std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &pts3D);

  cv::Point3d triangulateOnePoint(cv::Mat R, cv::Mat t, cv::Mat E, cv::Point2d p1, cv::Point2d p2);
  void triangulateSimple(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, std::vector<cv::Point3d> &pts3D, cv::Mat R0, cv::Mat t0, cv::Mat R1, cv::Mat t1, cv::Mat K);
  void triangulatePoints(cv::Mat projMatr1, cv::Mat projMatr2, std::vector<cv::Point2d> projPoints1, std::vector<cv::Point2d> projPoints2, std::vector<cv::Point3d> &points4D);
  void triangulate(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, std::vector<cv::Point3d> &pts3D, cv::Mat R0, cv::Mat t0, cv::Mat R1, cv::Mat t1, cv::Mat K);

  void computeExtrinsicParameters(std::vector<cv::Point2d> pts2D, std::vector<cv::Point3d> pts3D, cv::Mat K, cv::Mat &R, cv::Mat &t);
  void computeExtrinsicParameters2(std::vector<cv::Point2d> pts2D, std::vector<cv::Point3d> pts3D, cv::Mat K, cv::Mat &R, cv::Mat &t, bool useExtrinsicGuess);


  cv::Point2d projectPoint(cv::Mat R, cv::Mat t, cv::Mat K, cv::Point3d pt3D);
  std::vector<cv::Point2d> projectPoints(cv::Mat R, cv::Mat t, cv::Mat K, std::vector<cv::Point3d> pts3D);
  std::vector<cv::Point2d> projectPointsSimple(cv::Mat R, cv::Mat t, cv::Mat K, std::vector<cv::Point3d> pts3D);

  cv::Mat getRetropjImage(cv::Mat img, std::vector<cv::Point2d> pts_input, std::vector<cv::Point2d> pts_reproj, std::vector<bool> good_matching = std::vector<bool>(), bool displayAll = true, bool displayPointID = false);

  bool isForward(cv::Mat R, cv::Mat t, cv::Point3d pt);


private:
  bool isGood(cv::Point2d origin1, cv::Point2d reproj1, cv::Point2d origin2, cv::Point2d reproj2, double threshold, cv::Point3d pt3D, Pose pose1, Pose pose2);
};

#endif //RECONSTRUCTION3DSIMPLE_H
