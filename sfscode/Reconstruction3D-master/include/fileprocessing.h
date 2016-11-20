#ifndef FILEPROCESSING_H
#define FILEPROCESSING_H

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include "../include/color.h"

namespace FileProcessing{
  inline std::string valToStr(double d)
  {
      std::stringstream ss;
      ss << d;
      return ss.str();
  }

  inline bool fexists(std::string filename)
  {
      std::ifstream ifile(filename.c_str(), std::ifstream::in);
      return ifile;
  }

  inline std::string complete(int val, int nb_number)
  {
      std::stringstream ss;
      int nb_number_init = 1;
      while(val>=pow(10,nb_number_init)){
          nb_number_init++;
      }
      for(int i=nb_number_init;i<nb_number;i++)
          ss << 0;
      ss << val;
      return ss.str();
  }

  inline std::string makename(std::string base, int val, int nb_number)
  {
      std::stringstream ss;
      ss << base;
      ss << complete(val,nb_number);
      return ss.str();
  }

  inline std::string makefilename(std::string base, std::string ext, int val, int nb_number)
  {
      std::stringstream ss;
      ss << makename(base,val,nb_number);
      if(ext.size()==0)
          ext = ".jpg";
      if(ext.at(0)!='.')
          ss << ".";
      ss << ext;
      return ss.str();
  }

  inline int nbNumber(std::string base, std::string ext, int val, int max = 10)
  {
    for(int i=1;i<max;i++){
      if(fexists(makefilename(base,ext,val,i))){
        return i;
      }
    }
    return -1;
  }

  inline void saveMat(cv::MatExpr expr, std::string file, bool displayComment  = false, std::string comment = "")
  {
      cv::Mat M = expr;
      FILE *f = fopen(file.c_str(),"w");
      if(displayComment)
          fprintf(f,"%%%s\n",comment.c_str());
      for(int i=0;i<M.rows;i++){
          for(int j=0;j<M.cols;j++){
              fprintf(f,"%f ",M.at<double>(i,j));
          }
          fprintf(f,"\n");
      }
      fclose(f);
  }

  inline void saveMat(cv::Mat &M, std::string file, bool displayComment  = false, std::string comment = "")
  {
      FILE *f = fopen(file.c_str(),"w");
      if(displayComment)
          fprintf(f,"%%%s\n",comment.c_str());
      for(int i=0;i<M.rows;i++){
          for(int j=0;j<M.cols;j++){
              fprintf(f,"%f ",M.at<double>(i,j));
          }
          fprintf(f,"\n");
      }
      fclose(f);
  }

  inline void savePoints(std::string file, std::vector<cv::Point2d> pts)
  {
      cv::Mat M(pts.size(),2,CV_64F);
      for(unsigned int i=0;i<pts.size();i++){
          M.at<double>(i,0) = pts.at(i).x;
          M.at<double>(i,1) = pts.at(i).y;
      }
      std::stringstream ss;
      ss << pts.size();
      saveMat(M,file,true,ss.str());
  }

  inline void savePairsList(std::string file, std::vector< std::pair<int,int> > list)
  {
    FILE* f = fopen(file.c_str(),"w");
    int count = list.size();
    fprintf(f,"%%%d\n",count);
    for(int i=0;i<count;i++)
      fprintf(f,"%d %d\n",list.at(i).first,list.at(i).second);
    fclose(f);
  }

  inline void savePoints(std::string file, std::vector<cv::Point3d> pts, double clip = -1)
  {
      cv::Mat M(pts.size(),3,CV_64F);
      for(unsigned int i=0;i<pts.size();i++){
          if(cv::norm(pts.at(i))<clip || clip<0){
            M.at<double>(i,0) = pts.at(i).x;
            M.at<double>(i,1) = pts.at(i).y;
            M.at<double>(i,2) = pts.at(i).z;
          }else{
            M.at<double>(i,0) = 0.0;
            M.at<double>(i,1) = 0.0;
            M.at<double>(i,2) = 0.0;
          }
      }
      std::stringstream ss;
      ss << pts.size();
      saveMat(M,file,true,ss.str());
  }

  inline std::vector<cv::Point2d> readPoints(std::string file, bool &ok)
  {
      std::vector<cv::Point2d> pts; pts.clear();
      FILE *f = fopen(file.c_str(),"r");
      if(!f){
          std::cout << color::red() << "Can't read file : " << color::Red() << file << color::reset() << std::endl;
          ok = false;
          return pts;
      }
      int tot = 0;
      int res = fscanf(f,("%%%d\n"),&tot);
      if(res!=1){
          std::cout << color::red() << "Can't read number of points in : " << color::Red() << file << color::reset() << std::endl;
          ok = false;
          fclose(f);
          return pts;
      }

      float x,y;
      for(int i=0;i<tot;i++){
          fscanf(f,"%f %f\n",&x,&y);
          cv::Point2d pt(x,y);
          pts.push_back(pt);
      }
      fclose(f);
      ok = true;
      return pts;
  }

  inline std::vector<cv::Point2d> readPoints(std::string file)
  {
    bool ok;
    return readPoints(file,ok);
  }

  inline std::vector<cv::Point3d> readPoints3D(std::string file, bool &ok)
  {
      std::vector<cv::Point3d> pts; pts.clear();
      FILE *f = fopen(file.c_str(),"r");
      if(!f){
          std::cout << color::red() << "Can't read file : " << color::Red() << file << color::reset() << std::endl;
          ok = false;
          return pts;
      }
      int tot = 0;
      int res = fscanf(f,("%%%d\n"),&tot);
      if(res!=1){
          std::cout << color::red() << "Can't read number of points in : " << color::Red() << file  << color::reset() << std::endl;
          ok = false;
          fclose(f);
          return pts;
      }

      float x,y,z;
      for(int i=0;i<tot;i++){
          fscanf(f,"%f %f %f\n",&x,&y,&z);
          cv::Point3d pt(x,y,z);
          pts.push_back(pt);
      }
      fclose(f);
      ok = true;
      return pts;
  }

  inline std::vector<cv::Point3d> readPoints3D(std::string file)
  {
    bool ok;
    return readPoints3D(file,ok);
  }

  inline std::vector< std::pair<int,int> > readPairsList(std::string file, bool &ok)
  {
    ok = false;
    std::vector< std::pair<int,int> > list;
    FILE *f = fopen(file.c_str(),"r");
    if(!f){
      std::cout << color::red() << "Can't read file : " << color::Red() << file << color::reset() << std::endl;
        return list;
    }
    int tot = 0;
    int res = fscanf(f,("%%%d\n"),&tot);
    if(res!=1){
        std::cout << color::red() << "Can't read number of pairs in : " << color::Red() << file  << color::reset() << std::endl;
        fclose(f);
        return list;
    }

    int x,y;
    for(int i=0;i<tot;i++){
        fscanf(f,"%d %d\n",&x,&y);
        list.push_back( std::make_pair(x,y));
    }
    fclose(f);
    ok = true;
    return list;
  }
}

#endif // FILEPROCESSING_H
