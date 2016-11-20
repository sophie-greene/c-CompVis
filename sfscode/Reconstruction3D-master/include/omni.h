#ifndef OMNI_H
#define OMNI_H

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class Omni
{
public:
  Omni();
  cv::Point3d projectOnSphere(cv::Point2d pt, double epsilon);
  cv::Point2d projectPoint(cv::Point3d pt, cv::Mat R, cv::Mat t, double epsilon, cv::Mat K, cv::Mat dist = cv::Mat());

  void computeInitialPose(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat &R, cv::Mat &t, cv::Mat &K1, cv::Mat &dist1, double &epsilon1, cv::Mat &K2, cv::Mat &dist2, double &epsilon2);
  void solveRansacPnP(std::vector<cv::Point3d> &pts3D, std::vector<cv::Point2d> &pts2D, cv::Mat &R, cv::Mat &t, bool useExtrinsicGuess = false, int iterationsCount = 100, float reprojectionError = 8.0, int minInliersCount = 100, cv::OutputArray inliers = cv::noArray(), int flags = cv::ITERATIVE);

  cv::Mat computeFundamentalMatrix(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat &K1, cv::Mat &dist1, double &epsilon1, cv::Mat &K2, cv::Mat &dist2, double &epsilon2);
};


#endif // OMNI_H
