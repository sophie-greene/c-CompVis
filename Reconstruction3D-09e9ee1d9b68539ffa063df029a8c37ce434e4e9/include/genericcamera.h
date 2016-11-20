#ifndef GENERICCAMERA_H
#define GENERICCAMERA_H

#include <opencv2/opencv.hpp>

class GenericCamera
{
public:
  GenericCamera();
  GenericCamera(cv::Mat K, cv::Mat dist) {K_ = K; dist_ = dist;}

private:
  cv::Mat K_, dist_;
  cv::Mat R_, t_;
};


#endif // GENERICCAMERA_H
