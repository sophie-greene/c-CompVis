#include "../include/tools.h"
#include "gnuplot.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
  double clip = 25;
  bool permute_axis_output = true;
  cv::Mat permute = permute_axis_output?Tools::toRotationMatrix(M_PI/2,0,0):cv::Mat::eye(3,3,CV_64F);

  std::vector<cv::Point3d> pts;
  std::vector<cv::Mat> Rvec,tvec;

  double k_[9] = {673.1395959, 0, 316.6594553,
                  0, 676.0642576, 222.8059880,
                  0, 0, 1};
  cv::Mat K(3,3,CV_64F,k_);

  //Lecture du fichier Bdl
  ifstream Bdl(argv[1]);
  if (!Bdl.is_open())
    return -1;

  int NbImages;
  string VM,Acc,s;
  Bdl >> VM >> Acc;
  //Lecture de l'entete : taille des images et distortion
  int xBdl,yBdl;
  double rho;
  Bdl >> s >> xBdl >> s >> yBdl;
  Bdl >> s >> s >> Acc;
  Bdl >> s >> s >> s >> s;
  Bdl >> s >> s >> s >> s;
  Bdl >> s >> rho >> s >> s;
  Bdl >> Acc;
  int NbPIOut,NbPI;
  string s1,s2,s3,s4,s5,s6;
  Bdl >> s1 >> s2 >> s3 >> NbImages >> s5 >> s6;
  Bdl >> s >> NbPIOut >> s >> NbPI;
  Bdl >> s >> s >> s >> s >> s >> s >> s >> s;

  NbPI=NbPI+NbPIOut;

  //Lecture des points et cameras
  int NumeroPoint=0;	//Numero de stockage du point

  while (!Bdl.eof()){
    string Titre;
    int Numero;
    Bdl >> Titre >> Numero >> Acc;
    if (Titre=="Camera"){
      //cout << "Lecture camera " << Numero << flush;
      double m11,m12,m13,m14;
      double m21,m22,m23,m24;
      double m31,m32,m33,m34;
      int Flag;
      Bdl >> s >> s >> s >> Flag >> s >> s;
      Bdl >> m11 >> m12 >> m13 >> m14;
      Bdl >> m21 >> m22 >> m23 >> m24;
      Bdl >> m31 >> m32 >> m33 >> m34;
      Bdl >> Acc;
      if (Flag==1){
        double lambda=sqrt(m31*m31+m32*m32+m33*m33);
        cv::Mat R(3,3,CV_64F);
        R.at<double>(0,0) = m11/lambda;
        R.at<double>(0,1) = m12/lambda;
        R.at<double>(0,2) = m13/lambda;
        R.at<double>(1,0) = m21/lambda;
        R.at<double>(1,1) = m22/lambda;
        R.at<double>(1,2) = m23/lambda;
        R.at<double>(2,0) = m31/lambda;
        R.at<double>(2,1) = m32/lambda;
        R.at<double>(2,2) = m33/lambda;
        R = K.inv()*R;
        std::cout << R << std::endl;
        R *= 1.0/cv::norm(R);

        cv::Mat t(3,1,CV_64F);
        t.at<double>(0,0) = m14/lambda;
        t.at<double>(1,0) = m24/lambda;
        t.at<double>(2,0) = m34/lambda;
        t = K.inv()*t;

        t = -R.t()*t;
        R = R.t();

        Rvec.push_back(R);
        tvec.push_back(t);
      }
    }
    if (Titre=="Point"){
      //cout << "Lecture point " << Numero << flush;
      double Xw,Yw,Zw,Tw;
      Bdl >> Xw >> Yw >> Zw >> Tw;
      int Flag;
      Bdl >> s >> Flag >> s >> s;

      bool PointAStocker=( bool(Flag) );
      if (PointAStocker){
        cv::Point3d pt(Xw/Tw,Yw/Tw,Zw/Tw);
        pts.push_back(pt);
      }
      //Lecture des coordonnees images
      char c;
      Bdl >> c;
      while (c!='}'){
        Bdl.putback(c);
        int NumImage,Inlier;
        double xp,yp;
        Bdl >> NumImage >> Inlier >> xp >> yp;
        Bdl >> c;
        if (PointAStocker){
          cv::Point2d pt(xp,yp);
          if(Inlier){

          }else{

          }
        }
      }
      if (PointAStocker)
        NumeroPoint++;
    }
  }

  FILE *x = fopen("video_x.txt","w");
  FILE *y = fopen("video_y.txt","w");
  FILE *z = fopen("video_z.txt","w");

  cv::Mat ttot = cv::Mat::zeros(3,1,CV_64F);
  for(unsigned int i=0;i<Rvec.size();i++){
    cv::Mat Rt = Rvec.at(i).clone();
    cv::Mat tt = tvec.at(i).clone();
    gnuplot::displayMarker(Rt,tt,x,y,z,permute_axis_output);
    Tools::concatHorizontal(ttot,tvec.at(i),ttot);
  }

  fclose(x);
  fclose(y);
  fclose(z);

  gnuplot::openFile("path.txt");
  ttot = ttot.t()*permute;
  gnuplot::drawPath(ttot.t());
  gnuplot::closeFile();

  gnuplot::openFile("pts3D.txt");
  cv::Mat Rtemp = Tools::toMat(pts) * permute;
  pts = Tools::toVect3D(Rtemp);
  gnuplot::savePoints(pts,clip);
  gnuplot::closeFile();

  return 0;
}
