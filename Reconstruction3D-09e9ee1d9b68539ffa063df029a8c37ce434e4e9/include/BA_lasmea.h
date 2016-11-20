#ifndef BA_LASMEA_H
#define BA_LASMEA_H

#include <opencv2/core/core.hpp>
#include "bundle_adj_lib.hpp"
#include "config.h"
#include "mml.h"
#include "mparser.h"
#include "mwriter.h"
#include "tmpmemory.h"
#include <iostream>
#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>

tvmet::Matrix<flt,3,3> toTvMet33(cv::Mat &M){
  tvmet::Matrix<flt,3,3> out;
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      out(i,j) = M.at<double>(i,j);
    }
  }
  return out;
}

class BA_Lasmea
{
public:
  BA_Lasmea(std::vector<cv::Point3d> &points3D, std::vector<cv::Mat> &R, std::vector<cv::Mat> &t, std::vector<PointInfo> &points2D, cv::Mat &K){
    assert(R.size()==t.size());

    int maxLifeTime;
    float scF,lambda,threshold2D;
    MML lm(maxLifeTime,scF,lambda);
    TmpMemory mem(&lm, threshold2D, maxLifeTime);
    //MParser parser(inFileName, &ml, &mem, maxLifeTime);

    flt inlier2d = 100.00;
    int nPoints = points3D.size();
    int nCameras = R.size();

    mem.Init(nPoints);
    std::vector<std::vector<VectorCam<flt,2> > > capPixels;

    for(int i=0;i<points3D.size();i++){

      tvmet::Vector<flt,4> pt3D;

      pt3D(0) = points3D.at(i).x;
      pt3D(1) = points3D.at(i).y;
      pt3D(2) = points3D.at(i).z;
      pt3D(3) = 1.0;

      size_t param=1;
      int flag = 1;
      mem.AddnewPoint();
      mem.points[i].point = pt3D;
      mem.points[i].bFlag = (flag==0)?false:true;
      mem.points[i].param = param;

      std::vector<VectorCam<flt,2> > tempVec;// pour les points 2D

      for(int j=0;j<points2D.size();j++){
        if(i==points2D.at(j).ptIndex){
          VectorCam<flt,2> capPixel(points2D.at(j).camIndex);
          capPixel.vector(0) = points2D.at(j).pt2d.x;
          capPixel.vector(1) = points2D.at(j).pt2d.y;
          size_t camera = points2D.at(j).camIndex;
          size_t pFlag = 1;
          mem.points[i].Add2DPoint(capPixel.vector,camera,pFlag);
          tempVec.push_back(capPixel);
        }
      }
      capPixels.push_back(tempVec);
    }

    lm.Init(points3D.size(), nCameras);
    for (size_t cIdx=0; cIdx<R.size(); cIdx++) {
      lm.SetCameraInitValues(cIdx, t[cIdx].at<double>(0,0),t[cIdx].at<double>(1,0), t[cIdx].at<double>(2,0));
      tvmet::Matrix<flt,3,3> R0 = toTvMet33(R.at(cIdx));
      lm.SetR0(cIdx, R0);
      tvmet::Matrix<flt,3,3> K0 = toTvMet33(K);
      lm.SetK(cIdx, K0);
      lm.Yji[cIdx].clear();
    }

    int nCapPix = 0;
    for (size_t i1=0; i1<points3D.size(); i1++){
      lm.SetPointInitValues(i1, points3D[i1].x, points3D[i1].y, points3D[i1].z, 1.0);
      lm.SetNumProjCameras(i1, capPixels[i1].size());

      for (size_t i2=0; i2<capPixels[i1].size(); i2++){
          lm.AddCapturedPixel(i1, capPixels[i1][i2]);
          nCapPix++;
          lm.Yji[capPixels[i1][i2].cIdx].push_back( std::pair<int,int>(i1,i2));
        }
    }

    lm.SetNumCapPix(nCapPix);


    size_t it_interne = 25;
    int iterations = 50;
    int prevNum2DPoints = 0;
    for (size_t it=1; it<=iterations; it++) {
        std::cout<<"::iteration "<<it<<std::endl;
        if (it == 1) {
            std::cout<<inlier2d<<"% of inlier 2D points are used."<<std::endl;
        }
        else mem.UpdateML();
        if (lm.GetNum2DPoint() <= prevNum2DPoints && it >= 3) {
            std::cout<<"Number of inlier 2D points did not change."<<std::endl;
            break;
        }
        prevNum2DPoints = lm.GetNum2DPoint();
        std::cout<<"Number of inlier 3D points: "<<lm.GetNum3DPoint()<<std::endl;
        std::cout<<"Number of inlier 2D points: "<<lm.GetNum2DPoint()<<std::endl;
        lm.Run(it_interne);
        std::cout<<"Updating inliers."<<std::endl;
        mem.UpdatePoints();
    }
  }
};


#endif // BA_LASMEA_H
