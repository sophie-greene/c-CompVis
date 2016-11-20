#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"

namespace ImageProcessing{
  inline cv::Mat concatImages(cv::Mat &I1, cv::Mat &I2, double scale = -1, double horizontal = true, int maxWidth = -1)
  {
      cv::Mat I;
      if(I1.rows==0 || I1.cols==0){
          I = I2.clone();
          return I;
      }
      if(I2.rows==0 || I2.cols==0){
          I = I1.clone();
          return I;
      }
      cv::Size size;
      if(horizontal)
          size = cv::Size( I1.cols + I2.cols, MAX(I1.rows, I2.rows) );
      else
          size = cv::Size( MAX(I1.cols, I2.cols), I1.rows + I2.rows );

      I = cv::Mat::zeros( size, CV_MAKETYPE(I1.depth(), I1.channels()));

      cv::Mat outImg1, outImg2;
      if(horizontal){
          outImg1 = I( cv::Rect(0, 0, I1.cols, I1.rows) );
          outImg2 = I( cv::Rect(I1.cols, 0, I2.cols, I2.rows) );
      }else{
          outImg1 = I( cv::Rect(0, 0, I1.cols, I1.rows) );
          outImg2 = I( cv::Rect(0, I1.rows, I2.cols, I2.rows) );
      }

      I1.copyTo(outImg1);
      I2.copyTo(outImg2);

      if(scale>0 && scale!=1.0)
          cv::resize(I,I,cv::Size(I.cols*scale,I.rows*scale));

      if(maxWidth>0 && I.cols>maxWidth){
          double scale = (double)I.cols/(double)maxWidth;
          cv::resize(I,I,cv::Size(I.cols/scale,I.rows/scale));
      }
      return I;
  }

  inline std::string dim(cv::Mat &M)
  {
      std::stringstream ss;
      ss << M.rows << "x" << M.cols;
      return ss.str();
  }
}

#endif // IMAGEPROCESSING_H
