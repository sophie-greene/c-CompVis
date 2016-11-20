#ifndef POINTSMANAGER_H
#define POINTSMANAGER_H

#include <vector>
#include <opencv2/core/core.hpp>

#include <QMap>

/** @class PointsManager
* Define several method to organise 2D and 3D points correspondences\n
* This part will be replaced by using SOVIN software
*/

struct Image_Data{
  int id;
  cv::Mat image;
};

struct Point2D_Data{
  int id;
  cv::Point2d pt;
  cv::Mat descriptor;
  int image_ID;
  int previousPoint_ID;
};

struct Point3D_Data{
  int id;
  cv::Point3d pt;
  std::vector<int> pts2D_ID;
};


class PointsManager{
public:
  PointsManager(){
    point2D_incremental = 0;
    point3D_incremental = 0;
  }

  bool isExistsImage(int id){
    for(unsigned int i=0;i<imagesTable.size();i++)
      if(imagesTable.at(i).id==id)
        return true;
    return false;
  }

  bool addImage(int id, cv::Mat im){
    if(isExistsImage(id)){
      return false;
    }
    Image_Data image;
    image.id = id;
    image.image = im;
    imagesTable.push_back(image);
    return true;
  }

  bool isExistsPoint2D(int id){
    for(unsigned int i=0;i<point2DTable.size();i++)
      if(point2DTable.at(i).id==id)
        return true;
    return false;
  }

  Point2D_Data getPoint2d(int id){
    for(int i=0;i<point2DTable.size();i++){
        if(point2DTable.at(i).id==id)
          return point2DTable.at(i);
    }
    Point2D_Data p;
    p.id = -1;
    return p;
  }

  int addPoint2D(cv::Point2d pt, int imageID, int previousID = -1, cv::Mat descriptor = cv::Mat()){
    int id = getPoint2DID(imageID,pt);
    if(id>=0) return id;
    Point2D_Data pt2D;
    pt2D.id = point2D_incremental;
    pt2D.pt = pt;
    pt2D.descriptor = descriptor;
    pt2D.image_ID = imageID;
    pt2D.previousPoint_ID = previousID;
    point2DTable.push_back(pt2D);
    point2D_incremental++;
    return pt2D.id;
  }

  bool addPoint2D(std::vector<cv::Point2d> pts, int imageID, std::vector<int> previousIDs = std::vector<int>(), std::vector<cv::Mat> descriptors = std::vector<cv::Mat>()){
    for(int i=0;i<pts.size();i++){
        int previousID = -1;
        cv::Mat descriptor;
        if( i<previousIDs.size() ) previousID = previousIDs.at(i);
        if( i<descriptors.size() ) descriptor = descriptors.at(i);
        addPoint2D(pts.at(i), imageID, previousID, descriptor );
    }
    return true;
  }

  int getPoint2DID(int imageID, cv::Point2d pt){
    for(int i=point2DTable.size()-1;i>=0;i--){
      if(point2DTable.at(i).image_ID == imageID)
        if(point2DTable.at(i).pt == pt){
          return point2DTable.at(i).id;
      }
    }
    return -1;
  }
  int getPoint2DID(cv::Point2d pt,int imageID){
    for(int i=point2DTable.size()-1;i>=0;i--){
      if(point2DTable.at(i).image_ID == imageID)
        if(point2DTable.at(i).pt == pt){
          return point2DTable.at(i).id;
      }
    }
    return -1;
  }

  int isExistsPoint3D(cv::Point3d pt){
    for(int i=point3DTable.size()-1;i>=0;i--)
      if(point3DTable.at(i).pt==pt)
        return i;
    return -1;
  }

  bool isExistsPoint3D(int id){
    for(int i=point3DTable.size()-1;i>=0;i--)
      if(point3DTable.at(i).id==id)
        return true;
    return false;
  }

  bool addPoint3D(cv::Point3d pt, int point2D_ID = -1){
    int index = isExistsPoint3D(pt);
    if(index<0){
      Point3D_Data pt3D;
      pt3D.id = point3D_incremental;
      pt3D.pt = pt;
      if(point2D_ID>=0){
        pt3D.pts2D_ID.push_back(point2D_ID);
      }
      point3DTable.push_back(pt3D);
      point3D_incremental++;
    }else{
      if(point2D_ID<0)
        point3DTable[index].pts2D_ID.push_back(point2D_ID);
      return false;
    }
    return true;
  }

  bool add2D3DCorrespondance(int pt3D_id, int pt2D_id){
    if(!isExistsPoint3D(pt3D_id)){
        return false;
    }
    if(!isExistsPoint2D(pt2D_id)){
        return false;
    }
    for(unsigned int i=0;i<point3DTable.size();i++){
      if(point3DTable.at(i).id==pt3D_id){
        point3DTable[i].pts2D_ID.push_back(pt2D_id);
      }
    }
  }

  std::vector<cv::Point3d> getPoints3D(){
    std::vector<cv::Point3d> out;
    for(unsigned int i=0;i<point3DTable.size();i++){
      out.push_back(point3DTable.at(i).pt);
    }
    return out;
  }
  std::vector<cv::Point3d> getPoints3D(int occurance){
    std::vector<cv::Point3d> out;
    for(unsigned int i=0;i<point3DTable.size();i++){
      if(point3DTable.at(i).pts2D_ID.size()>=occurance)
        out.push_back(point3DTable.at(i).pt);
    }
    return out;
  }
   void setPoints3D(std::vector<cv::Point3d> pts){
     assert(point3DTable.size()==pts.size());
    for(unsigned int i=0;i<point3DTable.size();i++){
        point3DTable.at(i).pt = pts.at(i);
    }
  }

  cv::Point3d getPoints3D(int imageID, cv::Point2d pt){
    int id = getPoint2DID(imageID,pt);
    for(unsigned int i=0;i<point3DTable.size();i++){
      for(unsigned int j=0;j<point3DTable.at(i).pts2D_ID.size();j++){
        if(point3DTable.at(i).pts2D_ID.at(j)==id)
          return point3DTable.at(i).pt;
      }
    }
    return cv::Point3d(0,0,0);
  }

  cv::Point2d getPoint2D(Point3D_Data &point3D,int image, bool &found)
  {
    cv::Point2d pt(0,0);
    found = false;

    for(int i=0;i<point3D.pts2D_ID.size();i++){
      Point2D_Data  p = getPoint2d(point3D.pts2D_ID.at(i));
      if(p.id<0) continue;
      if(p.image_ID==image){
        found = true;
        pt = p.pt;
      }
    }

    return pt;
  }

  void getDatas(std::vector<cv::Point3d> &pts3D, std::vector<std::vector<cv::Point2d> > &pts2D, std::vector<std::vector<int> > &visibility)
  {
    std::cout << "Get " << point3DTable.size() << " 3D points" << std::endl;
    std::cout << "Get " << point2DTable.size() << " 2D points" << std::endl;
    std::cout << "Get " << imagesTable.size() << " images" << std::endl;
    pts3D.clear();
    pts2D.clear();
    visibility.clear();
    for(int i=0;i<point3DTable.size();i++){
      pts3D.push_back(point3DTable.at(i).pt);
    }
    for(int i=0;i<imagesTable.size();i++){
      if(i%10==0) std::cout << "." << std::flush;
      std::vector<cv::Point2d> pts2D_i;
      std::vector<int> visibility_i;
      for(int j=0;j<pts3D.size();j++){
        bool found = false;
        cv::Point2d pt = getPoint2D(point3DTable.at(j),i,found);
        if(found){
          pts2D_i.push_back(pt);
          visibility_i.push_back(1);
        }else{
          pts2D_i.push_back(pt);
          visibility_i.push_back(0);
        }
      }
      pts2D.push_back(pts2D_i);
      visibility.push_back(visibility_i);
    }
    std::cout << std::endl;
  }

  /* ---------------------------------------------------------------------- */

  int addPoint3D(cv::Point3d pt, int p1ID, int p2ID)
  {
    int index = getPoint3DID( p1ID ); //isExistsPoint3D(pt);
    if(index<0){
      Point3D_Data pt3D;
      pt3D.id = point3D_incremental;
      pt3D.pt = pt;
      if(p1ID>=0) pt3D.pts2D_ID.push_back(p1ID);
      if(p2ID>=0) pt3D.pts2D_ID.push_back(p2ID);
      point3DTable.push_back(pt3D);
      point3D_incremental++;
    }else{
      if(p1ID>=0) point3DTable[index].pts2D_ID.push_back(p1ID);
      if(p2ID>=0) point3DTable[index].pts2D_ID.push_back(p2ID);
    }
    return index;
  }

  int add(cv::Point2d &p1, cv::Point2d &p2, cv::Point3d &p3, int imageID)
  {
    int p1_ID = addPoint2D(p1,imageID-1);
    int p2_ID = addPoint2D(p2,imageID);

    return addPoint3D(p3,p1_ID,p2_ID);
  }

  void get(std::vector<cv::Point2d> ptsInput1, std::vector<cv::Point2d> ptsInput2, int imageID, std::vector<cv::Point2d> &pts2D_output, std::vector<cv::Point3d> &pts3D_output)
  {
    pts2D_output.clear();
    pts3D_output.clear();
    for(int i=0;i<ptsInput1.size();i++){
      int p1_ID = getPoint2DID( ptsInput1.at(i), imageID-1 );
      int p3_ID = getPoint3DID( p1_ID );

      if(p3_ID>=0){
        cv::Point2d pts2 = ptsInput2.at(i);
        cv::Point3d pts3 = getPoint3d(p3_ID);
        pts2D_output.push_back(pts2);
        pts3D_output.push_back(pts3);
      }
    }
  }

  cv::Point3d getPoint3d( int id )
  {
      for(int i=0;i<point3DTable.size();i++)
          if( point3DTable.at(i).id == id )
              return point3DTable.at(i).pt;
      return cv::Point3d(0,0,0);
  }

  int getPoint3DID( int p2DID )
  {
      if( p2DID<0 ) return -1;
      for(int i=point3DTable.size()-1;i>=0;i--){
        //if( contains(point3DTable[i].pts2D_ID,p2DID) )
        for(int j=point3DTable[i].pts2D_ID.size()-1;j>=0;j--){
          if(point3DTable[i].pts2D_ID.at(j)==p2DID){
             return point3DTable.at(i).id;
           }
        }
      }
      return -1;
  }

  bool contains(std::vector<int> &vect, int val)
  {
      for(int i=0;i<vect.size();i++)
          if(vect.at(i)==val)
              return true;
      return false;
  }

private:
  std::vector<Image_Data> imagesTable;
  std::vector<Point2D_Data> point2DTable;
  std::vector<Point3D_Data> point3DTable;
  int point2D_incremental;
  int point3D_incremental;

};


#endif // POINTSMANAGER_H
