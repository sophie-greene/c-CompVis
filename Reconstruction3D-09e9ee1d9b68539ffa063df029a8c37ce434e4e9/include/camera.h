#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core/core.hpp>


cv::Mat getCameraParam(double fx, double fy, double u0, double v0)
{
  double k_[9] = {fx, 0, u0,
                  0, fy, v0,
                  0,  0, 1};
  return cv::Mat(3,3,CV_64F,k_).clone();
}

cv::Mat getCameraParam(std::string camera, double fx = 500, double fy = 500, double u0 = 320, double v0 = 240)
{
  if(camera=="lasmea"){
    double k_[9] = {639.668700849,0, 518.777209138,
                    0, 639.642170319, 364.129721979,
                    0, 0, 1};
    return cv::Mat(3,3,CV_64F,k_).clone();
  }else if(camera=="laptop"){
    double k_[9] = {1272.316476804471, 0, 581.0803749147997,
                    0, 1270.506216271193, 508.8818531786451,
                    0, 0, 1};
    return cv::Mat(3,3,CV_64F,k_).clone();
  }else if(camera=="usb"){
    double k_[9] = {673.1395959858904, 0, 316.6594553502502,
                    0, 676.034257611742, 222.8059880708421,
                    0, 0, 1};
    return cv::Mat(3,3,CV_64F,k_).clone();
  }else if(camera=="pavin"){
    double k_[9] = {534.810881032, 0, 330.268815188,
                     0, 530.442799861, 222.442761123,
                     0, 0, 1};
    return cv::Mat(3,3,CV_64F,k_).clone();
  }

  return getCameraParam(fx,fy,u0,v0);
}


cv::Mat getCameraDistortion(std::string camera)
{
  if(camera=="lasmea"){
    double dist_[5] = {0.375566178043, 0.158153959529, 0.0645443925409, 0.0740136146723, 0.00701937630156}; //Eric Royer model
    return cv::Mat(1,5,CV_64F,dist_).clone();
  }else if(camera=="laptop"){
    return cv::Mat::zeros(1,5,CV_64F);
  }else if(camera=="usb"){
    double dist_[5] = {-0.02437508297644027, 0.3937559183298409, -0.01151686670629214, 0.007902746381030761, -1.282434437865985};
    return cv::Mat(1,5,CV_64F,dist_).clone();
  }else if(camera=="pavin"){
    double dist_[5] = {0.05175, -0.18348, -0.00026, -0.00018, 0.08880};
    return cv::Mat(1,5,CV_64F,dist_).clone();
  }else if(camera=="null"){
    return cv::Mat();
  }
  return cv::Mat::zeros(1,5,CV_64F);
}

#endif // CAMERA_H
