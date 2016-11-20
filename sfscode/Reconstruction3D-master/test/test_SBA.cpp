#include "../include/tools.h"
#include "../include/bundle_adjustment.h"
#include "../include/sba.h"
#include "../include/sba_ceres.h"
#include "../include/camera.h"
#include "../include/BA_lasmea.h"
#include "../include/datamanager.h"

#include "fileprocessing.h"
#include "gnuplot.h"
#include <time.h>
#include "../include/color.h"
#include <math.h>
#include <iostream>
#include <fstream>

#define CLOCKS_PER_MSEC (CLOCKS_PER_SEC/1000.0)

void displayHelp(std::string command)
{
  std::cout << "Usage:\n" << command << " " << std::endl;
}

int main(int argc, char* argv[])
{
  std::string input = "test_SBA_input_raw.txt";
  int iter = 100;

  for(int i=1;i<argc;i++){
    if(!strcmp(argv[i],"-n")) {iter = atoi(argv[i+1]); i++; continue;}
    if(!strcmp(argv[i],"-i")) {input = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-h")) {displayHelp(argv[0]); return 0;}
    std::cout << color::red() << "Unknown option : " << color::Red() << argv[i] << color::reset() << std::endl;
    displayHelp(argv[0]);
    return 1;
  }

  std::vector<cv::Point3d> points3D, points3D_res;
  std::vector<cv::Mat> Rvec, Rvec_res;
  std::vector<cv::Mat> tvec, tvec_res;
  std::vector<PointData> correspondances, correspondances_res;
  std::vector<cv::Mat> img;


  DataManager dm(input,true);
  dm.getPoses(Rvec,tvec);
  for(int i=0;i<Rvec.size();i++)
    Rvec[i] = Tools::Rodrigues(Rvec[i]);
  dm.getImages(img);
  dm.getPoints3D(points3D);
  dm.getPointData(correspondances);

  int nbPose = Rvec.size();
  int nbPoints = points3D.size();

  cv::Mat K = getCameraParam("pavin");
  cv::Mat dist = getCameraDistortion("pavin");

#if 0
  SBA_CERES sab_ceres(points3D,Rvec,tvec,correspondances,K,dist,iter);
  sab_ceres.getResult(points3D_res,Rvec_res,tvec_res,correspondances_res);
#else
  BundleAdjust sba;

  std::vector< std::vector<cv::Point2d> > points2D( nbPose, std::vector<cv::Point2d>(nbPoints, cv::Point2d(0,0)) );
  std::vector< std::vector<int> > visibility( nbPose, std::vector<int>(nbPoints, 0) );
  //cv::Mat visibility(nbPose,nbPoints,CV_8U);

  for(int i=0;i<correspondances.size();i++){
    int pointID = correspondances.at(i).getPtID();
    int poseID = correspondances.at(i).getPoseID();

    points2D[poseID][pointID] = correspondances.at(i).getPoint();
    visibility[poseID][pointID] = 1;
    //visibility.at<int>(poseID,pointID) = 1;
  }

  points3D_res = points3D;
  Rvec_res = Rvec;
  tvec_res = tvec;
  correspondances_res = correspondances;

  sba.bundleAdjustExtrinsic(points3D_res,points2D,visibility,K,dist,Rvec_res,tvec_res);

  for(int i=0;i<correspondances.size();i++){
    int pointID = correspondances.at(i).getPtID();
    int poseID = correspondances.at(i).getPoseID();
    cv::Point2d pt = Tools::projPoint(Rvec_res[poseID],tvec_res[poseID],K,dist,points3D[pointID]);
    correspondances_res[i].setPoint(pt);
  }

#endif

  FileProcessing::savePoints("SBA_in_pts.txt",points3D);
  gnuplot::savePoses("SBA_in",Rvec,tvec,true,1);
  FileProcessing::savePoints("SBA_out_pts.txt",points3D_res);
  gnuplot::savePoses("SBA_out",Rvec_res,tvec_res,true,1);


  for(int i=0;i<correspondances.size();i++){
    int id = correspondances.at(i).getPoseID();
    cv::Point2d pt1 = correspondances.at(i).getPoint();
    cv::Point2d pt2 = correspondances_res.at(i).getPoint();
    cv::line(img[id],pt1,pt2,CV_RGB(255,0,0));
    cv::circle(img[id],pt1,2,CV_RGB(0,0,255),-1);
    cv::circle(img[id],pt2,2,CV_RGB(0,255,0),-1);
  }

  for(int i=0;i<img.size();i++){
    std::stringstream str;
    str << "resultSBA/res" << i << ".jpg";
    cv::imwrite(str.str(),img[i]);
  }
}
