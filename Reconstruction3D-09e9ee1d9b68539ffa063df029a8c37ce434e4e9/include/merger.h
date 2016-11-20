#ifndef MERGER_H
#define MERGER_H

#include <vector>
#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/point_traits.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include "tools.h"

struct ImagePoint2D{
  cv::Point2d pt2d;
  cv::Mat descriptor;
};

struct MapPoint{
  cv::Point3d pt3d;
  std::vector<ImagePoint2D> pts2D;
};

class Map{
public:
  std::vector<MapPoint> pts;
  void transform(cv::Mat R, cv::Mat t, double scale){
    cv::Mat P(3,4,CV_64F);
    for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
        P.at<double>(i,j) = R.at<double>(i,j);
    for(int i=0;i<3;i++)
      P.at<double>(i,3) = t.at<double>(i,0);
    for(unsigned int i=0;i<pts.size();i++){
        cv::Mat pt = cv::Mat::ones(4,1,CV_64F);
        pt.at<double>(0,0) = pts.at(i).pt3d.x;
        pt.at<double>(1,0) = pts.at(i).pt3d.y;
        pt.at<double>(2,0) = pts.at(i).pt3d.z;
        pt *= scale;
        pt.at<double>(3,0) = 1.0;
        cv::Mat out = P*pt;
        pts[i].pt3d.x = out.at<double>(0,0);
        pts[i].pt3d.y = out.at<double>(1,0);
        pts[i].pt3d.z = out.at<double>(2,0);
    }
  }
  std::vector<cv::Point3d> getPoints3D(){
    std::vector<cv::Point3d> ptsOut;
    for(int i=0;i<pts.size();i++)
      ptsOut.push_back(pts.at(i).pt3d);
    return ptsOut;
  }
};

class Merger
{
public :
  Merger();
  ~Merger();

  cv::Mat computeICP(std::vector<cv::Point3d> pts1, std::vector<cv::Point3d> pts2, double distance = 10.0, double transformation_epsilon = 0.00001, int max_iterations = 100000);

  int computeDisplacement(std::vector<cv::Point3d> pts1, std::vector<cv::Point3d> pts2, cv::Mat &P, int subset_size = 5, int max_iter = 2500, double threshold = 0.01, double confidence = 0.95, bool random = false);
  int computeDisplacement(std::vector<cv::Point3d> pts1, std::vector<cv::Point3d> pts2, cv::Mat &R, cv::Mat &t, double &scale, int subset_size = 5, int max_iter = 2500, double threshold = 0.01, double confidence = 0.95, bool random = false);

  int computeMerging(Map &map1, Map &map2, cv::Mat &R, cv::Mat &tt, double &scale, Map &mapOut, std::vector< std::pair<int,int> > pairs = std::vector< std::pair<int,int> >());
  int compute3DMergingSolutions(Map &map1, Map &map2, std::vector<cv::Mat> &R, std::vector<cv::Mat> &t, std::vector< std::vector< std::pair<int,int> > > &pairs);
  int comfirm3DMergingSolutions(Map &map1, Map &map2, std::vector<cv::Mat> &R, std::vector<cv::Mat> &t, std::vector< std::vector< std::pair<int,int> > > &pairs);

  bool isMatching(cv::Mat d1, cv::Mat d2);

  void makePairs(Map &map1, Map &map2, std::vector< std::vector< std::pair<int,int> > > &pairs);

  int computeAffinePose(std::vector<cv::Point3d> pts1, std::vector<cv::Point3d> pts2, cv::Mat &R, cv::Mat &t, double &scale);
  int computePose(std::vector<cv::Point3d> pts1, std::vector<cv::Point3d> pts2, cv::Mat &R, cv::Mat &t, double &scale);
  //int computeMerging(std::vector<cv::Point3d> pts1, std::vector<cv::Point3d> pts2, std::vector<cv::Mat> descriptor1, std::vector<cv::Mat> descriptor2, cv::Mat &R, cv::Mat &t);
  //int compute3DMergingSolutions(std::vector<cv::Point3d> pts1, std::vector<cv::Point3d> pts2, std::vector<cv::Mat> &R, std::vector<cv::Mat> &t, std::vector< std::vector< std::pair<int,int> > > &pairs);
  //int comfirm3DMergingSolutions(std::vector<cv::Mat> descriptor1, std::vector<cv::Mat> descriptor2, std::vector<cv::Mat> &R, std::vector<cv::Mat> &t, std::vector< std::vector< std::pair<int,int> > > &pairs);
};

#endif // MERGER_H
