#include "../include/merger.h"
#include "../include/tools.h"

#include "fileprocessing.h"
#include "../include/color.h"

void displayHelp(std::string command)
{
  std::cout << "Usage:\n" << command << " -i1 <input1> -i2 <input2> [-p <pairs>] [-o <output>]" << std::endl;
}

int main(int argc, char* argv[])
{
  srand(time(0));
  std::string input1, input2, output = "merge_result.txt", pairsfile = "pairs.txt";

  bool usePairs = false;
  for(int i=1;i<argc;i++){
    if(!strcmp(argv[i],"-i1")) {input1 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-i2")) {input2 = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-o")) {output = argv[i+1]; i++; continue;}
    if(!strcmp(argv[i],"-p")) {pairsfile = argv[i+1]; i++; usePairs=true; continue;}
    std::cout << color("Unknown option : ",RED) << color(argv[i],RED,true) << std::endl;
    displayHelp(argv[0]);
    return 1;
  }
  if(input1==""||input2==""){
      displayHelp(argv[0]);
      return 1;
  }

  bool ok = false;
  std::vector<cv::Point3d> pts1 = readPoints3D(input1, ok); if(!ok) return 1;
  std::vector<cv::Point3d> pts2 = readPoints3D(input2, ok); if(!ok) return 1;
  std::vector< std::pair<int,int> > pairs;
  if(usePairs){
    pairs = readPairsList(pairsfile,ok);
    if(!ok) return 1;
  }

  Merger merger;
  std::vector<cv::Point3d> ptsout;
  cv::Mat R,t;
  double scale;
  if(usePairs){
      Map map1, map2, result;
      for(int i=0;i<pts1.size();i++){
        MapPoint mp;
        mp.pt3d = pts1.at(i);
        map1.pts.push_back(mp);
      }
      for(int i=0;i<pts2.size();i++){
        MapPoint mp;
        mp.pt3d = pts2.at(i);
        map2.pts.push_back(mp);
      }
    if(!merger.computeMerging(map1, map2, R, t, scale, result, pairs)){
      std::cout << "Error during merging" << std::endl;
      return 0;
    }
    for(int i=0;i<result.pts.size();i++){
        ptsout.push_back( result.pts.at(i).pt3d );
    }
  }else{
    int subset_size = 6;
    int max_iter = 10000;
    double threshold = 0.001;
    double confidence = 0.95;
    bool random = false;

    /*std::vector<cv::Point3d> pts1f,pts2f;
    for(int i=0;i<pts1.size();i++){
        cv::Point3f pt = pts1.at(i);
        pts1f.push_back(pt);
    }
    for(int i=0;i<pts2.size();i++){
        cv::Point3f pt = pts2.at(i);
        pts2f.push_back(pt);
    }

    cv::Mat A,inliers;
    cv::Mat P1 = Tools::toMat(pts1f).t();
    cv::Mat P2 = Tools::toMat(pts2f).t();
    cv::estimateAffine3D(P1,P2,A,inliers);
    Tools::display(A,"A");*/

    if(!merger.computeDisplacement(pts1,pts2, R, t, scale, subset_size, max_iter, threshold, confidence, random)){
      std::cout << "Error during merging" << std::endl;
      return 0;
    }
    ptsout = pts1;
    for(int i=0;i<pts2.size();i++){
        ptsout.push_back(scale*Tools::projPoint(R,t,pts2.at(i)));
    }
  }

  savePoints(output,ptsout);

  //cv::Mat P = merger.computeDisplacement(pts1,pts2);
  //Tools::display(P,"P");
  return 1;
}
