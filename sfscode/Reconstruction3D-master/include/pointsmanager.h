#ifndef POINTSMANAGER_H
#define POINTSMANAGER_H

#include <vector>
#include <opencv2/core/core.hpp>

/** @class PointsManager
* Define several method to organise 2D and 3D points correspondences\n
* This part will be replaced by using SOVIN software
*/

class PointsManager{
public:
  /** Add 3D and 2D points used in cuerrent image
   * @param pts3D is the input liste of 3D points
   * @param pts1 is the input liste of 2D points in image 1
   * @param pts2 is the input liste of 2D points in image 2
   * @param currentImage is the current image index
   * @param mask define if a point have to be added
   */
  void addPoints(std::vector<cv::Point3d> pts3D, std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, int currentImage, std::vector<bool> mask){
    int currentColumn = currentImage-1;
    if(currentColumn<=0){
       std::vector<cv::Point2d> l1,l2;
       for(unsigned int i=0;i<pts3D.size();i++){
         if(mask.at(i)){
           QVec.push_back(pts3D.at(i));
           l1.push_back(pts1.at(i));
           l2.push_back(pts2.at(i));
         }
       }
       im1PtsVec.push_back(l1);
       im2PtsVec.push_back(l2);
    }else{
        cv::Point pt0(-1,-1);
        int size = QVec.size();
        std::vector<cv::Point2d> l1(size,pt0),l2(size,pt0);
        //check correspondence
        for(unsigned int j=0;j<pts1.size();j++){
          if(mask.at(j)){
            int index = -1;
            for(int i=0;i<(int)(im2PtsVec.at(currentColumn-1).size());i++){
              if(im2PtsVec.at(currentColumn-1).at(i)==pts1.at(j)){
                index = i;
                break;
              }
            }
            if(index>=0){
              //Write point
                l1[index] = pts1.at(j);
                l2[index] = pts2.at(j);
            }else{
              //Add point
              QVec.push_back(pts3D.at(j));
              for(unsigned int k=0;k<im1PtsVec.size();k++){
                  im1PtsVec[k].push_back(pt0);
                  im2PtsVec[k].push_back(pt0);
              }
              l1.push_back(pts1.at(j));
              l2.push_back(pts2.at(j));
            }
          }
        }
        im1PtsVec.push_back(l1);
        im2PtsVec.push_back(l2);
    }
  }

  /** Get 3D points corresponding with image points
   * @param imageIndex
   * @param pts1,pts2
   * @param pts2d_out
   * @param pts3d_out
   */
  void get3Dcorrespondences(int imageIndex, std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, std::vector<cv::Point2d> &pts2d_out, std::vector<cv::Point3d> &pts3d_out){
    int columnIndex = imageIndex-1;
    pts2d_out.clear();
    pts3d_out.clear();
    for(unsigned int i=0;i<pts1.size();i++){
      for(unsigned int j=0;j<im2PtsVec.at(columnIndex).size();j++){
        if(pts1.at(i)==im2PtsVec.at(columnIndex).at(j) && im2PtsVec.at(columnIndex).at(j).x>=0){
          pts3d_out.push_back(QVec.at(j));
          pts2d_out.push_back(pts2.at(i));
        }
      }
    }
  }

  /** Get points visibility in each image
   */
  std::vector<std::vector<int> > getVisibility(){
    int countImages = im1PtsVec.size();
    std::vector<std::vector<int> > out;
    for(unsigned int i=0;i<countImages;i++){
      std::vector<int> temp;
      for(unsigned int j=0;j<im1PtsVec.at(i).size();j++){
        if(im1PtsVec.at(i).at(j).x<0 || im1PtsVec.at(i).at(j).y<0){
          temp.push_back(0);
        }else{
          temp.push_back(1);
        }
      }
      out.push_back(temp);
    }
    std::vector<int> temp;
    for(unsigned int j=0;j<im2PtsVec.at(countImages-1).size();j++){
      if(im2PtsVec.at(countImages-1).at(j).x<0 || im2PtsVec.at(countImages-1).at(j).y<0){
        temp.push_back(0);
      }else{
        temp.push_back(1);
      }
    }
    out.push_back(temp);
    return out;
  }

  /** Get all 2D points saved
   */
  std::vector<std::vector<cv::Point2d> > getPoints2D(){
    std::vector<std::vector<cv::Point2d> > out;
    for(unsigned int i=0;i<im1PtsVec.size();i++){
      out.push_back(im1PtsVec.at(i));
    }
    out.push_back(im2PtsVec.at(im2PtsVec.size()-1));
    return out;
  }

  /** Get 3D points corresponding with image several points in one image
   * @param pts is input list of 2D points in image 1
   * @param index is the image index
   */
  std::vector<cv::Point3d> getPoints3D(std::vector<cv::Point2d> pts, int index){
    assert(index>=0 && index<im1PtsVec.size());
    std::vector<cv::Point3d> p3;
    for(int j=0;j<pts.size();j++){
      bool found = false;
      for(int i=0;i<im1PtsVec.at(index).size();i++){
        if(pts.at(j)==im1PtsVec.at(index).at(i)){
          p3.push_back(QVec.at(i));
          found = true;
        }
      }
      if(!found)
        p3.push_back(cv::Point3d(0,0,0));
    }
    return p3;
  }

  /** Get all 3D points
   */
  std::vector<cv::Point3d> getPoints3D(){
    return QVec;
  }

  /** Set all 3D points
   * @param p is the input list of 3D points
   */
  void setPoints3D(std::vector<cv::Point3d> p){
    QVec.clear();
    QVec = p;
  }
private:
  std::vector<cv::Point3d> QVec;
  std::vector< std::vector<cv::Point2d> > im1PtsVec;
  std::vector< std::vector<cv::Point2d> > im2PtsVec;
};


#endif // POINTSMANAGER_H
