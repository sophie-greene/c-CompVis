#include "../include/merger.h"
#include "../include/tools.h"

#include "fileprocessing.h"
#include "../include/color.h"


void displayHelp(std::string path)
{
  std::cout << path << " -i1 input_file1 -i2 input_file2 [-o output_file]" << std::endl;
}

int main(int argc, char* argv[])
{
  std::string input1, input2, output;
  double tx=0,ty=0,tz=0;
  double rx=0,ry=0,rz=0;

  for(int i=1;i<argc;i++){
    if(!strcmp(argv[i],"-i1")) {input1 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-i2")) {input2 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-o")) {output = argv[i+1]; i++; continue;}
    std::cout << color("Unknown option : ",RED) << color(argv[i],RED,true) << std::endl;
    displayHelp(argv[0]);
    return 1;
  }
  if(input1=="" || input2==""){
    std::cout << color("No input file",RED) << std::endl;
    displayHelp(argv[0]);
    return 1;
  }

  bool ok = false;
  std::vector<cv::Point3d> pts1 = readPoints3D(input1, ok); if(!ok) return 1;
  std::vector<cv::Point3d> pts2 = readPoints3D(input2, ok); if(!ok) return 1;

  Merger merger;
  cv::Mat T = merger.computeICP(pts1,pts2);

  Tools::display(T,"T");

  cv::Mat R = T.colRange(0,3).rowRange(0,3).clone();
  cv::Mat Rv = Tools::Rodrigues(R);
  cv::Mat t = T.col(3).rowRange(0,3).clone();

  Tools::display(R,"R");
  Tools::display(Rv,"Rv");
  Tools::display(t,"t");

  if(output!=""){
    std::vector<cv::Point3d> cloud_out;

    for(int i=0;i<pts1.size();i++){
      cloud_out.push_back(Tools::projPoint(R,t,pts1.at(i)));
    }

    savePoints(output,cloud_out);
  }
}

