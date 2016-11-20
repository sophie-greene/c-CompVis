#ifndef TOOLS_H
#define TOOLS_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

namespace Tools{

  //Image manipulation
  enum FLIPMODE {NONE = -2, BOTH_AXIS, HORIZONTAL_AXIS, VERTICAL_AXIS };
  cv::Mat flip(cv::Mat &M, FLIPMODE mode = VERTICAL_AXIS);
  cv::Mat rescale(cv::Mat &M, double scale);
  cv::Mat unwarp(cv::Mat &img, cv::Point2d center, double R0, double R1, double theta, double phi, int w, int h, FLIPMODE flip_mode = NONE, cv::OutputArray mapx_out = cv::noArray(), cv::OutputArray mapy_out = cv::noArray());

  //Angle manipulation
  double toRad(double deg);
  double toDeg(double rad);

  //Point transformation
  cv::Mat toMat(std::vector<cv::Point2d> &pts);
  cv::Mat toMat(std::vector<cv::Point3d> &pts);
  cv::Mat toMat(cv::Point3d Q);
  cv::Mat toMat(cv::Point2d Q);
  cv::Mat toHomogeneous(std::vector<cv::Point2d> &pts);
  cv::Mat toHomogeneous(std::vector<cv::Point3d> &pts);
  cv::Mat toHomogeneous(cv::Point3d Q);
  cv::Mat toHomogeneous(cv::Point2d Q);
  cv::Point2d toPoint2d(cv::Mat &m);
  cv::Point3d toPoint3d(cv::Mat &m);
  std::vector<cv::Point2d> toVect2D(cv::Mat &M);
  std::vector<cv::Point3d> toVect3D(cv::Mat &M);
  std::vector<cv::Point2f> toFloatVector(std::vector<cv::Point2d> &pts);
  std::vector<cv::Point3f> toFloatVector(std::vector<cv::Point3d> &pts);

  //Tools
  int sign(double x);
  cv::Scalar toScalar(cv::Mat M);
  std::string toStr(double d);

  //Matrix value
  void set(cv::Mat &M, int r, int c, double val);
  double get(cv::Mat &M, int r, int c);
  double trace(cv::Mat &M);
  cv::Mat copy(cv::Mat M);

  //Matrix formation
  void concatHorizontal(cv::Mat &M1, cv::Mat &M2, cv::Mat &Mout);
  void concatVertical(cv::Mat &M1, cv::Mat &M2, cv::Mat &Mout);
  cv::Mat concatV(cv::Mat M1, cv::Mat M2, cv::Mat M3 = cv::Mat(), cv::Mat M4 = cv::Mat());
  cv::Mat concatH(cv::Mat M1, cv::Mat M2, cv::Mat M3 = cv::Mat(), cv::Mat M4 = cv::Mat());
  void diag(cv::Mat &d, cv::Mat &D);
  void diag(double v1, double v2, double v3, cv::Mat &D);
  cv::Mat diag(cv::Mat &d);
  cv::Mat diag(double v1, double v2, double v3);
  void skewSymetric(cv::Mat &v, cv::Mat &S);
  cv::Mat skewSymetric(cv::Mat v);
  void skewSymetric(double v1, double v2, double v3, cv::Mat &S);
  cv::Mat skewSymetric(double v1, double v2, double v3);
  void toRank2(cv::Mat &M);

  //Matrix manipulation
  void multiply(cv::Mat &M1, cv::Mat &M2, cv::Mat &Mout);
  void multiply(cv::Mat &M1, cv::Mat &M2, cv::Mat &M3,cv::Mat &Mout);

  //Math
  cv::Mat leftNullVector(cv::Mat A);
  cv::Mat rightNullVector(cv::Mat A);
  cv::Mat minEigenVector(cv::Mat &M);
  cv::Mat kron(cv::Mat M1, cv::Mat M2);
  cv::Mat pseudoInv(cv::Mat &M);
  cv::Mat solvePseudoInverse(cv::Mat A,cv::Mat b);

  //Display
  void display(double value, std::string name = "", bool jump = false);
  void display(CvMat m, std::string name = "", bool jump = false);
  void display(cv::Mat m, std::string name = "", bool jump = false);
  void display(cv::MatExpr m, std::string name = "", bool jump = false);
  void displaySize(cv::Mat m, std::string name = "");
  void displaySize(cv::MatExpr m, std::string name = "");

  //Display Image
  void displayImage(cv::Mat image, bool waitKey = true, std::string windowTitle = "Display", bool closeWindow = true);

  //Lift
  cv::Mat liftCoordinate(cv::Point2d pt, int size = 4);
  cv::Mat liftCoordinate(cv::Point3d pt, int size = 10);
  cv::Mat liftMatrix(cv::Mat M);

  //Normalize
  void normalize(std::vector<cv::Point2d> &pts1,std::vector<cv::Point2d> &pts2, cv::Mat &K1, cv::Mat &K2, std::vector<cv::Point2d> &pts1n,std::vector<cv::Point2d> &pts2n);
  void normalize(std::vector<cv::Point2d> &pts, cv::Mat &K, std::vector<cv::Point2d> &ptsn);
  void normalize(std::vector<cv::Point2d> &pts, cv::Mat &K, cv::Mat &ptsn);
  void normalize(cv::Mat Min, cv::Mat &K, cv::Mat &Mout);
  cv::Mat normalize(std::vector<cv::Point2d> &pts, cv::Mat &K);

  //Projection
  cv::Point3d projPoint(cv::MatExpr R, cv::MatExpr t, cv::Point3d Q);
  cv::Point3d projPoint(cv::Mat &R, cv::Mat &t, cv::Point3d Q);
  cv::Point2d projPoint(cv::Mat &R, cv::Mat &t, cv::Mat K, cv::Mat dist, cv::Point3d Q);
  std::vector<cv::Point2d> projPoints(cv::Mat &R, cv::Mat &t, cv::Mat K, cv::Mat dist, std::vector<cv::Point3d> pts);
  std::vector<cv::Point3d> projPoints3D(cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> pts, double scale = 1.0);

  //Operation
  cv::Mat quaternion(double a, double b, double c, double d);
  cv::Mat Rodrigues(double x, double y, double z);
  cv::Mat Rodrigues(cv::Mat R);
  cv::Point3d crossProduct(cv::Point3d u, cv::Point3d v);

  //Test
  bool isRotationMatrix(cv::Mat M);
  bool areEqual(cv::Mat &M1, cv::Mat &M2, double threshold);

  //Flip image points
  void flipX(std::vector<cv::Point2d> &pts, double size);
  void flipY(std::vector<cv::Point2d> &pts, double size);

  //Usefull matrices
  cv::Mat toTranslationMatrix(double x, double y, double z);
  cv::Mat toRotationMatrix(double x, double y, double z);
  cv::Mat toRotationMatrixX(double x);
  cv::Mat toRotationMatrixY(double y);
  cv::Mat toRotationMatrixZ(double z);
  cv::Mat createMat33(double v1, double v2, double v3, double v4, double v5, double v6, double v7, double v8, double v9);
  cv::Mat createMat41(double v1, double v2, double v3, double v4);
  cv::Mat createMat31(double v1, double v2, double v3);

  //Line intersection
  cv::Point3d linesIntersection(cv::Point3d P1, cv::Point3d P2, cv::Point3d P3 ,cv::Point3d P4);
  cv::Point3d linePlanIntersection(cv::Point3d P1, cv::Point3d P2, cv::Point3d u, cv::Point3d v, cv::Point3d A);

  //Mean
  cv::Mat meanCols(cv::Mat &M);
  cv::Mat meanRows(cv::Mat &M);

  //Subset
  bool contains(std::vector<int> &v, int n);
  void getSubset(std::vector<cv::Point2d> v1, std::vector<cv::Point2d> &o1, std::vector<cv::Point2d> v2, std::vector<cv::Point2d> &o2, int nb);
  void getSubset(std::vector<cv::Point3d> v1, std::vector<cv::Point3d> &o1, std::vector<cv::Point3d> v2, std::vector<cv::Point3d> &o2, int nb);
  void getSubset(std::vector<cv::Point3d> v3, std::vector<cv::Point3d> &o3, std::vector<cv::Point2d> v2, std::vector<cv::Point2d> &o2, int nb);
  void getSubset(std::vector<cv::Point3d> v3, std::vector<cv::Point3d> &o3, int nb);
  void getSubset(std::vector<cv::Point2d> v2, std::vector<cv::Point2d> &o2, int nb);

  //Point tools
  int getNearestPoint3D(std::vector<cv::Point3d> &l, cv::Point3d &p);
  int getNearestPoint2D(std::vector<cv::Point2d> &l, cv::Point2d &p);

}

#endif // TOOLS_H
