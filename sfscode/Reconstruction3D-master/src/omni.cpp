#include "../include/omni.h"

Omni::Omni()
{

}

cv::Point3d Omni::projectOnSphere(cv::Point2d pt, double epsilon)
{
  double x = pt.x;
  double y = pt.y;
  double gamma = sqrt(1+(1-epsilon*epsilon)*(x*x+y*y));
  double nu = (-gamma-epsilon*(x*x+y*y))/(epsilon*epsilon*(x*x+y*y)-1);
  cv::Point3d xbar(x,y,1/(epsilon*nu));
  return (1/nu + epsilon)*xbar;
}

cv::Point2d Omni::projectPoint(cv::Point3d pt, cv::Mat R, cv::Mat t, double epsilon, cv::Mat K, cv::Mat dist){
  cv::Point3d sphereCenter;
  cv::Point3d p = pt-sphereCenter;
  p = p * (1.0 / (double) cv::norm(p));
  std::vector<cv::Point3d> vecIn;
  vecIn.push_back( p );
  std::vector<cv::Point2d> vecOut;
  cv::projectPoints(vecIn,R,t,K,dist,vecOut);
  return vecOut.at(0);
}

void Omni::computeInitialPose(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat &R, cv::Mat &t, cv::Mat &K1, cv::Mat &dist1, double &epsilon1, cv::Mat &K2, cv::Mat &dist2, double &epsilon2)
{
  cv::Mat F = computeFundamentalMatrix(pts1,pts2,K1,dist1,epsilon1,K2,dist2,epsilon2);

  //cv::Mat E = computeEssentialMatrix(F,cam1,cam2);

  cv::Mat Rvec, tvec;
  //extractPose(R,Rvec,tvec);

}

//void Omni::solvePnPRansac(std::vector<cv::Point3d> &obj, std::vector<cv::Point2d> &im, cv::Mat &R, cv::Mat &t, bool useExtrinsicGuess, int iterationsCount, float reprojectionError, int minInliersCount, cv::OutputArray inliers, int flags)
//{
//}

cv::Mat Omni::computeFundamentalMatrix(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat &K1, cv::Mat &dist1, double &epsilon1, cv::Mat &K2, cv::Mat &dist2, double &epsilon2)
{
  cv::Mat F;

  return F;
}
