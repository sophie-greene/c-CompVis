#ifndef LOOPCLOSURE_H
#define LOOPCLOSURE_H

#include <iostream>
#include "datamanager.h"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "../include/timer.h"

struct Node{
  int id;
  cv::Mat descriptor;
  std::vector<PointData> data;
};

class LoopClosure
{
public:
  LoopClosure(DataManager *dm);

  void createInvertedFileFromDB();
  void addPoint(const cv::Mat &pointDescriptor, const PointData &point);

  std::vector<int> getNodeID(const cv::Mat &currentDescriptor);

private:
  DataManager *dm_;

  std::vector<Node> invertedFile;

  cv::Mat getDescriptor(const cv::Mat &img, const cv::Point2d &pt);
  std::vector<cv::Mat> getDescriptors(const cv::Mat &img, const std::vector<cv::Point2d> &pts);

  bool isMatching(const cv::Mat &descriptor1, const cv::Mat &descriptor2);

  cv::DescriptorMatcher *matcher;
  cv::DescriptorExtractor *extractor;

};

#endif // LOOPCLOSURE_H
