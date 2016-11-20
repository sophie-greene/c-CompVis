#ifndef MATLAB_H
#define MATLAB_H

#include <string>
#include <opencv2/core/core.hpp>

namespace matlab{
    FILE * fmatlab;
    void openFile(std::string filename)
    {
        fmatlab = fopen(filename.c_str(),"w");
    }

    void closeFile()
    {
        fclose(fmatlab);
    }

    void exportMatrix(std::string name, cv::Mat &M)
    {
      fprintf(fmatlab,"%s=[\n",name.c_str());
      for(int i=0;i<M.rows;i++){
        for(int j=0;j<M.cols;j++){
          fprintf(fmatlab,"%f",M.at<double>(i,j));
          if(j+1<M.cols)
            fprintf(fmatlab,",");
        }
        fprintf(fmatlab,";\n");
      }
      fprintf(fmatlab,"];\n");
    }

    void exportMatrix(std::string name, std::vector<cv::Point2d> &vec)
    {
      fprintf(fmatlab,"%s=[\n",name.c_str());
      for(unsigned int i=0;i<vec.size();i++){
        fprintf(fmatlab,"%f,%f;\n",vec.at(i).x,vec.at(i).y);
      }
      fprintf(fmatlab,"];\n");
    }

    void exportPoint(std::string name, cv::Point3d p)
    {
      fprintf(fmatlab,"%s=[\n",name.c_str());
      fprintf(fmatlab,"%f,%f,%f;\n",p.x,p.y,p.z);
      fprintf(fmatlab,"];\n");
    }

    void exportMatrix(std::string name, std::vector<cv::Point3d> &vec, double scale = 1.0)
    {
      fprintf(fmatlab,"%s=[\n",name.c_str());
      for(unsigned int i=0;i<vec.size();i++){
        fprintf(fmatlab,"%f,%f,%f;\n",vec.at(i).x*scale,vec.at(i).y*scale,vec.at(i).z*scale);
      }
      fprintf(fmatlab,"];\n");
    }

    cv::Point3d projPoint(cv::Mat &R, cv::Mat &, cv::Point3d Q)
    {
      cv::Point3d pt;
      /*Q.x -= t.at<double>(0,0);
        Q.y -= t.at<double>(1,0);
        Q.z -= t.at<double>(2,0);*/
      pt.x = Q.x * R.at<double>(0,0) + Q.y * R.at<double>(0,1) + Q.z * R.at<double>(0,2);
      pt.y = Q.x * R.at<double>(1,0) + Q.y * R.at<double>(1,1) + Q.z * R.at<double>(1,2);
      pt.z = Q.x * R.at<double>(2,0) + Q.y * R.at<double>(2,1) + Q.z * R.at<double>(2,2);
      return pt;
    }

    void displayLine(cv::Point3d p1, cv::Point3d p2, std::string color = "r", std::string option = "")
    {
      if(option.size()>0)
        option = ","+option;
      fprintf(fmatlab,"plot3([%f,%f],[%f,%f],[%f,%f],'%s'%s);\n", p1.x,p2.x, p1.y,p2.y, p1.z,p2.z, color.c_str(),option.c_str());
    }

    void displayMarker(cv::Mat &R, cv::Mat &t, int number = -1, int color = 0)
    {
      fprintf(fmatlab,"hold on\n");
      cv::Point3d O(t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0));
      cv::Point3d p1(1,0,0);
      cv::Point3d p2(0,1,0);
      cv::Point3d p3(0,0,1);
      cv::Mat origin(3,1,CV_64F);
      for(int i=0;i<3;i++) origin.at<double>(i,0) = 0.0;
      cv::Point3d Q1 = projPoint(R,origin,p1) + O;
      cv::Point3d Q2 = projPoint(R,origin,p2) + O;
      cv::Point3d Q3 = projPoint(R,origin,p3) + O;
      if(color==0){
        displayLine(O,Q1,"r","'LineWidth',2");
        displayLine(O,Q2,"g","'LineWidth',2");
        displayLine(O,Q3,"b","'LineWidth',2");
      }else{
        displayLine(O,Q1,"--r");
        displayLine(O,Q2,"--g");
        displayLine(O,Q3,"--b");
      }
      if(number>=0)
           fprintf(fmatlab,"text(%f,%f,%f,'%d','HorizontalAlignment','left')\n",O.x,O.y,O.z+0.1,number);
    }
}

#endif // MATLAB_H
