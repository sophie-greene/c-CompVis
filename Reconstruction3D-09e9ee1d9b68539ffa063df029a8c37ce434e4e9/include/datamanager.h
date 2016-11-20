#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <fstream>


//PointData
class PointData{
public:
  PointData(cv::Point2d pt, int poseID, int ptID) : pt_(pt), poseID_(poseID), ptID_(ptID) {}
  cv::Point2d getPoint() { return pt_; }
  int getPoseID() { return poseID_; }
  int getPtID() { return ptID_; }

  void setPoint(cv::Point2d pt) { pt_ = pt; }

private:
  cv::Point2d pt_;
  int poseID_;
  int ptID_;
};

//PoseData
class PoseData{
public:
  PoseData(cv::Mat R, cv::Mat t, int id, std::string path, std::vector<int> pointIDs = std::vector<int>()) : id_(id), path_(path) {
    R_ = R.clone();
    t_ = t.clone();
    pointIDs_ = pointIDs;
    //if(pointIDs.size()>0)
    //  pointIDs_.insert(pointIDs.begin(),pointIDs.end());
  }
  cv::Mat getR() {return R_;}
  cv::Mat getT() {return t_;}
  int getID() {return id_;}
  std::string getPath() {return path_;}

  void addPointID(int pointID) { pointIDs_.push_back(pointID); }
  void addPointsID(std::vector<int> pointIDs) {  if(pointIDs.size()>0)  pointIDs_.insert(pointIDs.end(),pointIDs.begin(),pointIDs.end()); }

  int getNbPoint() { return pointIDs_.size(); }
  int getPointID(int n) { return pointIDs_.at(n); }

private:
  cv::Mat R_,t_;
  int id_;
  std::string path_;
  std::vector<int> pointIDs_;
};

//DataManager
class DataManager
{
public:
  DataManager(bool verbose = false) : verbose_(verbose) {}
  DataManager(std::string filepath, bool verbose = false) : verbose_(verbose) { load(filepath); }

  void setVerbose(bool state){ verbose_ = state; }
  void setK(cv::Mat &K){ K_ = K; }
  void setDist(cv::Mat &dist){ dist_ = dist; }
  void setKDist(cv::Mat &K, cv::Mat &dist){ K_ = K; dist_ = dist; }

  int addPoint(PointData pt){
    int id = pts2D_.size();
    pts2D_.push_back( pt );
    return id;
  }
  int addPoint(cv::Point2d pt, int poseID, int ptID){
    int id = pts2D_.size();
    pts2D_.push_back( PointData(pt,poseID,ptID) );
    return id;
  }
  int addPoint3d(cv::Point3d pt){
    int id = pts3D_.size();
    pts3D_.push_back(pt);
    return id;
  }
  int addPose(cv::Mat R, cv::Mat t, int imageID = 0, std::string path = ""){
    int id = poses_.size();
    PoseData pose(R,t,imageID,path);
    poses_.push_back(pose);
    return id;
  }

  void getImages(std::vector<cv::Mat> &vec)
  {
    vec.clear();
    for(int i=0;i<poses_.size();i++){
      cv::Mat img = cv::imread(poses_.at(i).getPath());
      vec.push_back( img );
    }
  }

  std::vector<cv::Mat> getImages()
  {
    std::vector<cv::Mat> vec;
    for(int i=0;i<poses_.size();i++){
      cv::Mat img = cv::imread(poses_.at(i).getPath());
      vec.push_back( img );
    }
    return vec;
  }

  std::vector<PoseData> getPosesData()
  {
    return poses_;
  }

  std::vector<PointData> getPointData()
  {
    return pts2D_;
  }

  void getPointData(std::vector<PointData> &vec)
  {
    vec = pts2D_;
  }

  void getCorrespondances(std::vector<PointData> &vec)
  {
    vec = pts2D_;
  }

  std::vector<cv::Point3d> getPoints3D(){
    return pts3D_;
  }

  void getPoints3D(std::vector<cv::Point3d> &vec){
    vec = pts3D_;
  }

  cv::Point3d getPoint3D(int id){
    if(id<0 || id>=pts3D_.size())
      return cv::Point3d();
    return pts3D_.at(id);
  }

  int getNbPose(){
    return poses_.size();
  }

  int getPreviousPoseID(){
    int id = poses_.size();
    return id - 1;
  }

  int getCurrentPoseID(){
    int id = poses_.size();
    return id;
  }

  int getNextPoseID(){
    int id = poses_.size();
    return id + 1;
  }

  int getNbPoint(){
    return pts3D_.size();
  }

  int getNbObservation(){
    return pts2D_.size();
  }

  void writeMinimizator(std::string filename)
  {
    std::ofstream ofs(filename.c_str());

    ofs << "cameras" << std::endl;
    ofs << "1" << std::endl;
    ofs << "0 1" << std::endl;
    ofs << "0 0 PinholeCameraWithDirectDistortion" << std::endl;
    ofs << K_.at<double>(0,0) << std::endl;
    ofs << K_.at<double>(0,2) << std::endl;
    ofs << K_.at<double>(1,2) << std::endl;
    ofs << dist_.at<double>(0,0) << std::endl;
    ofs << dist_.at<double>(0,1) << std::endl;
    ofs << dist_.at<double>(0,4) << std::endl;
    ofs << dist_.at<double>(0,2) << std::endl;
    ofs << dist_.at<double>(0,3) << std::endl;
    for(int i=0;i<2;i++)
      ofs << "0.0 0.0 0.0" << std::endl;

    ofs << "poses" << std::endl;
    ofs << "0 " << poses_.size() << std::endl;
    for(int i=0;i<poses_.size();i++){
      ofs << "0 " << i << std::endl;
      PoseData pose = poses_.at(i);
      cv::Mat R = pose.getR();
      cv::Mat t = pose.getT();
      cv::Mat R33;
      cv::Rodrigues(R,R33);
      R33 = R33.t();
      t = -R33*t;
      cv::Rodrigues(R33,R);

      ofs << R.at<double>(0,0) << " " << R.at<double>(0,1) << " " << R.at<double>(0,2) << std::endl;
      ofs << t.at<double>(0,0) << " " << t.at<double>(1,0) << " " << t.at<double>(2,0) << std::endl;
    }

    ofs << "3Dpoints" << std::endl;
    ofs << pts3D_.size() << std::endl;
    for(int i=0;i<pts3D_.size();i++)
      ofs << pts3D_.at(i).x << " " << pts3D_.at(i).y << " " << pts3D_.at(i).z << std::endl;

    ofs << "observations" << std::endl;
    ofs << pts2D_.size() << std::endl;
    for(int i=0;i<pts2D_.size();i++){
      ofs << "0 0 " << pts2D_.at(i).getPoseID() << " " << pts2D_.at(i).getPtID() << " ";
      ofs << pts2D_.at(i).getPoint().x << " " << pts2D_.at(i).getPoint().y << std::endl;
    }

    ofs << "imageNames" << std::endl;
    ofs << "0" << std::endl;

  }

  void writeBDL(std::string filename)
  {
    //int nbPoses = poses_.size();
    //int nbPoints = pts3D_.size();
    //int nbObservations = pts2D_.size();

    int nbPoses = 5;
    int nbPoints = 0;
    int nbObservations = 0;

    std::ofstream ofs(filename.c_str());
    ofs << "xImageSize " << 640 << std::endl;
    ofs << "yImageSize " << 480 << std::endl;
    ofs << "xMax " << 640 << std::endl;
    ofs << "yMax " << 480 << std::endl;
    ofs << "xCenter " << 320 << std::endl;
    ofs << "yCenter " << 240 << std::endl;
    ofs << "Distorsion1 " << 0 << std::endl;
    ofs << "AspectRatio " << 1 << std::endl;
    ofs << "ArePointOk " << 0 << std::endl;
    ofs << "NbViewOk " << 0 << std::endl;
    ofs << "NbPointOk " << 0 << std::endl;
    ofs << "NbInlier " << 0 << std::endl;
    ofs << "NbOutlier " << 0 << std::endl;
    ofs << "MaxSquareErrorDistance " << 1 << std::endl;
    ofs << "Lambda " << 1e-10 << std::endl;
    ofs << "OldChi2 " << 100 << std::endl;
    ofs << "NewChi2 " << 100 << std::endl << std::endl;

    //cv::Mat K;
    for(int i=0;i<nbPoses;i++){
      int flag = 0, param = 0;
      ofs << "Camera " << i << " " << flag << " " << param << std::endl;
      //cv::Mat R = poses_.at(i).getR();
      //cv::Mat t = poses_.at(i).getT();

      cv::Mat M = cv::Mat::zeros(3,4,CV_64F);

      for(int m=0;m<3;m++){
        for(int n=0;n<4;n++){
          ofs << M.at<double>(m,n);
          if(n!=3)
            ofs << " ";
        }
        ofs << std::endl;
      }
      ofs << std::endl;
    }
  }

  void write(std::string filename)
  {
    FILE *file = fopen(filename.c_str(),"w");
    int nbPoses = poses_.size();
    int nbPoints = pts3D_.size();
    int nbObservations = pts2D_.size();

    fprintf(file,"4 5\n");
    fprintf(file,"%f %f %f %f\n",K_.at<double>(0,0),K_.at<double>(1,1),K_.at<double>(0,2),K_.at<double>(1,2));
    fprintf(file,"%f %f %f %f %f\n",dist_.at<double>(0,0),dist_.at<double>(0,1),dist_.at<double>(0,2),dist_.at<double>(0,3),dist_.at<double>(0,4));

    fprintf(file,"%d %d %d\n",nbPoses,nbObservations,nbPoints);

    for(int i=0;i<nbPoses;i++){
      cv::Mat R = poses_.at(i).getR();
      cv::Mat t = poses_.at(i).getT();
      for(int j=0;j<3;j++){
        fprintf(file,"%lf ",R.at<double>(0,j));
      }
      for(int j=0;j<3;j++){
        fprintf(file,"%lf ",t.at<double>(j,0));
      }
      fprintf(file,"%d %s\n",poses_.at(i).getID(),poses_.at(i).getPath().c_str());
    }

    for(int i=0;i<nbObservations;i++){
      cv::Point2d pt = pts2D_.at(i).getPoint();
      int poseIndex = pts2D_.at(i).getPoseID();
      int pointIndex = pts2D_.at(i).getPtID();
      fprintf(file,"%d %d %lf %lf\n",poseIndex,pointIndex,pt.x,pt.y);
    }

    for(int i=0;i<nbPoints;i++){
      cv::Point3d pt = pts3D_.at(i);
      fprintf(file,"%lf %lf %lf\n",pt.x,pt.y,pt.z);
    }

    fclose(file);
  }

  void load(std::string filename)
  {
    poses_.clear();
    pts2D_.clear();
    pts3D_.clear();
    K_ = cv::Mat::eye(3,3,CV_64F);

    std::ifstream str ( filename.c_str() , std::ifstream::in );

    int nbIntrisic = 0, nbDistorsion = 0;
    str >> nbIntrisic >> nbDistorsion;
    dist_ = cv::Mat::zeros(nbDistorsion,1,CV_64F);

    {
      double fx, fy, d, u, v;
      d = 0.0;
      if(nbIntrisic==3){
        str >> fx >> u >> v;
        fy = fx;
      }else if(nbIntrisic==4){
        str >> fx >> fy >> u >> v;
      }else if(nbIntrisic==5){
        str >> fx >> fy >> u >> v >> d;
      }else{
        std::cout << "Invalid number of intrinsic parameters" << std::endl;
      }

      K_.at<double>(0,0) = fx;
      K_.at<double>(1,1) = fy;
      K_.at<double>(0,2) = u;
      K_.at<double>(1,2) = v;
      K_.at<double>(0,1) = d;
    }

    {
      for(int i=0;i<nbDistorsion;i++){
        double d;
        str >> d;
        dist_.at<double>(0,i) = d;
      }
    }

    int nbPoses = 0, nbPoints = 0, nbObservations = 0;
    str >> nbPoses >> nbObservations >> nbPoints;

    if(verbose_){
      std::cout << "Loading " << filename << " ("<<nbPoses<<" poses, "<<nbObservations<<" pts2D, "<<nbPoints<<" pts3D"<<") ..." << std::endl;
    }

    for(int i=0;i<nbPoses;i++){
      double x,y,z;
      str >> x >> y >> z;
      cv::Mat R(3,1,CV_64F);
      R.at<double>(0,0) = x;
      R.at<double>(0,1) = y;
      R.at<double>(0,2) = z;
      str >> x >> y >> z;
      cv::Mat t(3,1,CV_64F);
      t.at<double>(0,0) = x;
      t.at<double>(1,0) = y;
      t.at<double>(2,0) = z;
      //R_.push_back(R);
      //t_.push_back(t);
      int id;
      std::string path;
      str >> id >> path;
      PoseData pose(R,t,i,path);
      poses_.push_back(pose);
    }

    for(int i=0;i<nbObservations;i++){
      int poseIndex,pointIndex;
      double x,y;
      str >> poseIndex >> pointIndex >> x >> y;

      for(int j=0;j<poses_.size();j++){
        if(poses_.at(j).getID()==poseIndex){
          poses_[j].addPointID(pts2D_.size());
          break;
        }
      }

      PointData p(cv::Point2d(x,y),poseIndex,pointIndex);
      pts2D_.push_back(p);
    }

    for(int i=0;i<nbPoints;i++){
      double x,y,z;
      str >> x >> y >> z;
      cv::Point3d pt(x,y,z);
      pts3D_.push_back( pt );
    }

    if(verbose_){
      std::cout << "Data loaded :" << std::endl;
      std::cout << " - " << poses_.size() << " poses" << std::endl;
      std::cout << " - " << pts2D_.size() << " 2D points" << std::endl;
      std::cout << " - " << pts3D_.size() << " 3D points" << std::endl;

      std::cout << "K = " << K_ << std::endl;
      std::cout << "dist = " << dist_ << std::endl;

      //std::cout << "Files : " << std::endl;
      //for(int i=0;i<poses_.size();i++)
      //  std::cout << " (" << poses_.at(i).getID() << ") " << poses_.at(i).getPath() << std::endl;
    }

  }

  void getPoses(std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec)
  {
    Rvec.clear();
    tvec.clear();
    for(int i=0;i<poses_.size();i++){
      Rvec.push_back( poses_.at(i).getR() );
      tvec.push_back( poses_.at(i).getT() );
    }
  }

  void getPose(int n, cv::Mat &R, cv::Mat &t)
  {
    R = poses_.at(n).getR();
    t = poses_.at(n).getT();
  }

  void getLastPose(cv::Mat &R, cv::Mat &t)
  {
    int n = poses_.size()-1;
    getPose(n,R,t);
  }

  void printStats()
  {
    std::map<int,int> mapPose;
    std::map<int,int> mapPoint;
    std::map<int,int> mapPointNb;
    for(int i=0;i<pts2D_.size();i++){
      int poseID = pts2D_[i].getPoseID();
      int ptID = pts2D_[i].getPtID();

      mapPose[poseID]++;
      mapPoint[ptID]++;
    }
    FILE *fout = fopen("mapPose.txt","w");
    for(std::map<int,int>::iterator it = mapPose.begin() ; it != mapPose.end() ; ++it){
      //std::cout << "clef: " << it->first << " val: " << it->second << std::endl;
      fprintf(fout,"%d %d\n",it->first,it->second);
    }
    fclose(fout);
    fout = fopen("mapPoint.txt","w");
    for(std::map<int,int>::iterator it = mapPoint.begin() ; it != mapPoint.end() ; ++it){
      //std::cout << "clef: " << it->first << " val: " << it->second << std::endl;
      fprintf(fout,"%d %d\n",it->first,it->second);
      mapPointNb[it->second]++;
    }
    fclose(fout);
    fout = fopen("mapPointNb.txt","w");
    for(std::map<int,int>::iterator it = mapPointNb.begin() ; it != mapPointNb.end() ; ++it){
      //std::cout << "clef: " << it->first << " val: " << it->second << std::endl;
      fprintf(fout,"%d %d\n",it->first,it->second);
    }
    fclose(fout);
  }

private:
  std::vector<PoseData> poses_;
  std::vector<cv::Point3d> pts3D_;
  std::vector<PointData> pts2D_;
  bool verbose_;
  cv::Mat K_,dist_;

};

#endif // DATAMANAGER_H
