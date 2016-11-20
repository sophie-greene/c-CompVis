#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <string>
#include <opencv2/core/core.hpp>

namespace gnuplot{
    static FILE * fgnuplot;
    inline void openFile(std::string filename)
    {
        fgnuplot = fopen(filename.c_str(),"w");
    }

    inline void closeFile()
    {
        fclose(fgnuplot);
    }

    inline void exportMatrix(std::string name, cv::Mat &M)
    {
      fprintf(fgnuplot,"%s=[\n",name.c_str());
      for(int i=0;i<M.rows;i++){
        for(int j=0;j<M.cols;j++){
          fprintf(fgnuplot,"%f",M.at<double>(i,j));
          if(j+1<M.cols)
            fprintf(fgnuplot,",");
        }
        fprintf(fgnuplot,";\n");
      }
      fprintf(fgnuplot,"];\n");
    }

    inline void exportMatrix(std::string name, std::vector<cv::Point2d> &vec)
    {
      fprintf(fgnuplot,"%s=[\n",name.c_str());
      for(unsigned int i=0;i<vec.size();i++){
        fprintf(fgnuplot,"%f,%f;\n",vec.at(i).x,vec.at(i).y);
      }
      fprintf(fgnuplot,"];\n");
    }

   inline  void exportPoint(std::string name, cv::Point3d p)
    {
      fprintf(fgnuplot,"%s=[\n",name.c_str());
      fprintf(fgnuplot,"%f,%f,%f;\n",p.x,p.y,p.z);
      fprintf(fgnuplot,"];\n");
    }

    inline void exportMatrix(std::string name, std::vector<cv::Point3d> &vec, double scale = 1.0)
    {
      fprintf(fgnuplot,"%s=[\n",name.c_str());
      for(unsigned int i=0;i<vec.size();i++){
        fprintf(fgnuplot,"%f,%f,%f;\n",vec.at(i).x*scale,vec.at(i).y*scale,vec.at(i).z*scale);
      }
      fprintf(fgnuplot,"];\n");
    }

    inline cv::Point3d projPoint(cv::Mat &R, cv::Mat &t, cv::Point3d Q)
    {
      cv::Point3d pt;
      pt.x = Q.x * R.at<double>(0,0) + Q.y * R.at<double>(0,1) + Q.z * R.at<double>(0,2);
      pt.y = Q.x * R.at<double>(1,0) + Q.y * R.at<double>(1,1) + Q.z * R.at<double>(1,2);
      pt.z = Q.x * R.at<double>(2,0) + Q.y * R.at<double>(2,1) + Q.z * R.at<double>(2,2);
      pt.x += t.at<double>(0,0);
      pt.y += t.at<double>(1,0);
      pt.z += t.at<double>(2,0);
      return pt;
    }

    inline void displayLine(cv::Point3d p1, cv::Point3d p2, bool switch_axes = true, double clip = 2000)
    {
      if(cv::norm(p1-p2)<clip){
        if(switch_axes)
          fprintf(fgnuplot,"%f %f %f\n%f %f %f\n\n\n", p1.x, p1.z, -p1.y, p2.x, p2.z, -p2.y);
        else
          fprintf(fgnuplot,"%f %f %f\n%f %f %f\n\n\n", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
      }
    }

    inline void displayLine(cv::Point3d p1, cv::Point3d p2, FILE* file, bool switch_axes = true)
    {
      if(switch_axes)
        fprintf(file,"%f %f %f\n%f %f %f\n\n\n", p1.x, p1.z, -p1.y, p2.x, p2.z, -p2.y);
      else
        fprintf(file,"%f %f %f\n%f %f %f\n\n\n", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
    }

    inline void displayMarker(cv::Mat &R, cv::Mat &t, bool switch_axes)
    {
      cv::Point3d O(t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0));
      cv::Point3d p1(1,0,0);
      cv::Point3d p2(0,1,0);
      cv::Point3d p3(0,0,1);
      cv::Point3d Q1 = projPoint(R,t,p1);
      cv::Point3d Q2 = projPoint(R,t,p2);
      cv::Point3d Q3 = projPoint(R,t,p3);
      displayLine(O,Q1,switch_axes);
      displayLine(O,Q2,switch_axes);
      displayLine(O,Q3,switch_axes);
    }

    inline void displayMarker(cv::Mat &R, cv::Mat &t, FILE* x, FILE* y, FILE* z, bool switch_axes = true, double size = 1.0)
    {
      cv::Point3d O(t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0));
      cv::Point3d p1(size,0,0);
      cv::Point3d p2(0,size,0);
      cv::Point3d p3(0,0,size);
      cv::Point3d Q1 = projPoint(R,t,p1);
      cv::Point3d Q2 = projPoint(R,t,p2);
      cv::Point3d Q3 = projPoint(R,t,p3);
      displayLine(O,Q1,x,switch_axes);
      displayLine(O,Q2,y,switch_axes);
      displayLine(O,Q3,z,switch_axes);
    }

    inline void drawPath(cv::Mat M)
    {
        for(int i=0;i<M.cols;i++){
            //warning, permute axis
            fprintf(fgnuplot,"%f %f %f\n", M.at<double>(0,i), M.at<double>(1,i), M.at<double>(2,i));
        }
        fprintf(fgnuplot,"\n");
    }

    inline void savePoses(std::string filename_base, std::vector<cv::Mat> Rvec, std::vector<cv::Mat> tvec, bool switch_axes = false, double size = 1.0)
    {
      std::stringstream ss_x,ss_y,ss_z,ss_path;
      ss_x << filename_base << "_x.txt";
      ss_y << filename_base << "_y.txt";
      ss_z << filename_base << "_z.txt";
      ss_path << filename_base << "_path.txt";
      FILE *x = fopen(ss_x.str().c_str(),"w");
      FILE *y = fopen(ss_y.str().c_str(),"w");
      FILE *z = fopen(ss_z.str().c_str(),"w");
      FILE *fpath = fopen(ss_path.str().c_str(),"w");

      for(unsigned int i=0;i<Rvec.size();i++){
        cv::Mat Rt = Rvec.at(i).clone();
        cv::Mat tt = tvec.at(i).clone();
        gnuplot::displayMarker(Rt,tt,x,y,z,switch_axes,size);

        if(switch_axes)
          fprintf(fpath,"%f %f %f\n",tvec.at(i).at<double>(0,0),tvec.at(i).at<double>(2,0),-tvec.at(i).at<double>(1,0));
        else
          fprintf(fpath,"%f %f %f\n",tvec.at(i).at<double>(0,0),tvec.at(i).at<double>(1,0),tvec.at(i).at<double>(2,0));
      }

      fclose(x);
      fclose(y);
      fclose(z);
      fclose(fpath);
    }

    inline void drawLine(cv::Point3d p1, cv::Point3d p2){
      fprintf(fgnuplot,"%f %f %f\n%f %f %f\n\n", p1.x,p1.y,p1.z, p2.x,p2.y,p2.z);
    }
    inline void drawQuad(cv::Point3d p1, cv::Point3d p2, cv::Point3d p3, cv::Point3d p4){
      drawLine(p1,p2);
      drawLine(p2,p3);
      drawLine(p3,p4);
      drawLine(p4,p1);
      fprintf(fgnuplot,"\n");
    }
    inline void drawTri(cv::Point3d p1, cv::Point3d p2, cv::Point3d p3){
      drawLine(p1,p2);
      drawLine(p2,p3);
      drawLine(p3,p1);
      fprintf(fgnuplot,"\n");
    }
    inline void drawPath(std::vector<cv::Point3d> pts){
      for(int i=0;i<pts.size()-1;i++)
        drawLine(pts.at(i),pts.at(i+1));
      if(pts.size()>0)
        drawLine(pts.at(pts.size()-1),pts.at(0));
      fprintf(fgnuplot,"\n");
    }

    inline void saveMat(cv::Mat &M, bool displayComment  = false, std::string comment = "")
    {
        if(displayComment)
            fprintf(fgnuplot,"%%%s\n",comment.c_str());
        for(int i=0;i<M.rows;i++){
            for(int j=0;j<M.cols;j++){
                fprintf(fgnuplot,"%f ",M.at<double>(i,j));
            }
            fprintf(fgnuplot,"\n");
        }
    }

    inline void savePoints(std::vector<cv::Point2d> pts)
    {
        cv::Mat M(pts.size(),2,CV_64F);
        for(unsigned int i=0;i<pts.size();i++){
            M.at<double>(i,0) = pts.at(i).x;
            M.at<double>(i,1) = pts.at(i).y;
        }
        std::stringstream ss;
        ss << pts.size();
        saveMat(M,true,ss.str());
    }

    inline void savePoints(std::vector<cv::Point2d> pts, double clip)
    {
      if(clip<0){
        savePoints(pts);
        return;
      }
      std::vector<cv::Point2d> pts_out;
      for(unsigned int i=0;i<pts.size();i++){
        if(cv::norm(pts.at(i))<clip)
          pts_out.push_back(pts.at(i));
      }
      savePoints(pts_out);
    }

    inline void savePoints(std::vector<cv::Point3d> pts)
    {
        cv::Mat M(pts.size(),3,CV_64F);
        for(unsigned int i=0;i<pts.size();i++){
            M.at<double>(i,0) = pts.at(i).x;
            M.at<double>(i,1) = pts.at(i).y;
            M.at<double>(i,2) = pts.at(i).z;
        }
        std::stringstream ss;
        ss << pts.size();
        saveMat(M,true,ss.str());
    }

    inline void savePoints(std::vector<cv::Point3d> pts, double clip)
    {
      if(clip<0){
        savePoints(pts);
        return;
      }
      std::vector<cv::Point3d> pts_out;
      for(unsigned int i=0;i<pts.size();i++){
        if(cv::norm(pts.at(i))<clip)
          pts_out.push_back(pts.at(i));
      }
      savePoints(pts_out);
    }

    inline void savePoint3DID(std::vector<cv::Point3d> pts){
      for(int i=0;i<pts.size();i++){
        fprintf(fgnuplot,"set label %d \"%d\" at %f,%f,%f left\n",i+1,i,pts.at(i).x,pts.at(i).y,pts.at(i).z);
      }
    }

    inline void draw(std::string filename, std::string plot, int width = 1024, int height = 768, double xr_min = 0, double xr_max = 0, double yr_min = 0, double yr_max = 0, double zr_min = 0, double zr_max = 0, bool setXYZequal = true){
      FILE *drawGnuplot;
      drawGnuplot = fopen("temp.gnup","w");
      fprintf(drawGnuplot,"set term png size %d,%d\n",width<0?1024:width,height<0?768:height);
      fprintf(drawGnuplot,"set out \"%s\"\n",filename.c_str());
      if(xr_min==xr_max){
         fprintf(drawGnuplot,"set autoscale x\n");
      }else{
         fprintf(drawGnuplot,"set xr [%f:%f]\n",xr_min,xr_max);
      }
      if(yr_min==yr_max){
        fprintf(drawGnuplot,"set autoscale y\n");
      }else{
        fprintf(drawGnuplot,"set yr [%f:%f]\n",yr_min,yr_max);
      }
      if(zr_min==zr_max){
        fprintf(drawGnuplot,"set autoscale z\n");
      }else{
        fprintf(drawGnuplot,"set zr [%f:%f]\n",zr_min,zr_max);
      }
      fprintf(drawGnuplot,"unset label\n");
      if(setXYZequal) fprintf(drawGnuplot,"set view equal xyz\n");
      fprintf(drawGnuplot,"%s\n",plot.c_str());
      fclose(drawGnuplot);
      system("gnuplot temp.gnup");
    }
}

#endif // GNUPLOT_H
