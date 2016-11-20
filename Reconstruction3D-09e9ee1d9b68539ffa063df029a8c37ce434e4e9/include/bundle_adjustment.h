#ifndef BUNDLEAJUSTEMENT_H
#define BUNDLEAJUSTEMENT_H

#include <opencv/cv.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include "opencv2/stitching/detail/matchers.hpp"
#include "../include/tools.h"
#include "../include/datamanager.h"

#include <mml.h>
#include <tmpmemory.h>


class PointInfo{
public:
  PointInfo(cv::Point2d pt, int cam, int pt3D) : pt2d(pt), camIndex(cam), ptIndex(pt3D)
  {}

  cv::Point2d pt2d;
  int camIndex;
  int ptIndex;
};


class BundleAdjust
{
public:
  BundleAdjust();

  tvmet::Matrix<flt,3,3> toTvMet33(cv::Mat &M){
    tvmet::Matrix<flt,3,3> out;
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        out(i,j) = M.at<double>(i,j);
      }
    }
    return out;
  }

  tvmet::Matrix<flt,3,1> toTvMet31(cv::Mat &M){
    tvmet::Matrix<flt,3,1> out;
    for(int i=0;i<3;i++){
      out(i,0) = M.at<double>(i,0);
    }
    return out;
  }


  void bundleAdjust( std::vector<cv::Point3d>& points, std::vector<PointData>& imagePoints, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& R, std::vector<cv::Mat>& t, int max_iter = 30);

  void bundleAdjust( std::vector<cv::Point3d>& points, std::vector<std::vector<cv::Point2d> >& imagePoints, std::vector<std::vector<int> >& visibility,
                     cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& R, std::vector<cv::Mat>& t, int max_iter = 30);
  void bundleAdjustExtrinsic( std::vector<cv::Point3d>& points, std::vector<std::vector<cv::Point2d> >& imagePoints, std::vector<std::vector<int> >& visibility,
                      cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& R, std::vector<cv::Mat>& t, int max_iter = 30);

  void bundleAdjustOpenCV( std::vector<cv::Point3d>& points, std::vector<std::vector<cv::Point2d> >& imagePoints, std::vector<std::vector<int> >& visibility,
                           cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& R, std::vector<cv::Mat>& t, int max_iter = 30);

  void bundleAdjustLM( std::vector<cv::Point3d>& points, std::vector<std::vector<cv::Point2d> >& imagePoints, std::vector<std::vector<int> >& visibility,
                       cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& R, std::vector<cv::Mat>& t, int max_iter = 30);

private:
  //cv::Mat _K;
  //void my_fjac(int /*i*/, int /*j*/, CvMat *point_params, CvMat* cam_params, CvMat* A, CvMat* B, void* /*data*/);
  //void my_func(int /*i*/, int /*j*/, CvMat *point_params, CvMat* cam_params, CvMat* estim, void* /*data*/);
  //void my_fjac_ba(int i, int j, cv::Mat& point_params, cv::Mat& cam_params, cv::Mat& A, cv::Mat& B, void* data);
  //void my_func_ba(int i, int j, cv::Mat& point_params, cv::Mat& cam_params, cv::Mat& estim, void* data);
};

#endif  //BUNDLEAJUSTEMENT_H
