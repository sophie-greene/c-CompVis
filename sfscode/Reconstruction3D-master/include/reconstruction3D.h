#ifndef RECONSTRUCTION3D_H
#define RECONSTRUCTION3D_H

/**
* @author Léo Baudouin\n
* @em baudouin.leo@gmail.com
*/

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv/cv.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <stdio.h>

#include "estimator.h"

#include "tools.h"

#include <omp.h>

using namespace Tools;

//TODO, remove that
#define USE_ESSENTIAL_MATRIX 1

/** @class Reconstruction3D
* @short Define several method to reconstruct 3D view from multiple view geometry
*/
class Reconstruction3D
{
public:
   /** @enum CameraType
   * Define camera types:\n
   * PERSP represents all perspective cases\n
   * AFFINE represents all affine cases\n
   * OMNI reprensents all omnidirectional cases
   */
   enum CameraType {PERSP=0, AFFINE, OMNI};

   /** @enum ErrorType*/
   enum ErrorType {NO_ERROR=0, MAT_FUNDAMENTAL_ERROR, MAT_ESSENTIAL_ERROR, EXTRACT_EXTRINSIC_ERROR, FIND_EXTRINSIC_ERROR, MAT_CONCAT_ERROR, TRIANGULATION_ERROR, REPROJECTION_ERROR, NO_GOOD_POINTS_ERROR, EXTRINSIC_NOT_FOUND, UNKNOWN_CAMERA_PAIR, NOT_ENOUGH_POINTS, UNNAMED_ERROR};

   /** @struct Camera
   *    Define camera type and parameters
   */
   struct Camera{
       CameraType type; /**< type is the CameraType*/
       cv::Mat K; /**< K is the Projection matrix*/
       cv::Mat dist; /**< dist is the Distortion matrix*/
       double epsilon; /**< epsilon is one spherical model parameter*/
       double phi; /**< phi is one spherical model parameter*/
       bool undistImage; /**< undistImage define if the image have to be undistorded before computing*/
   };

   /** @struct Param
   *    Define estimator parameters
   */
   struct Param{
       int maxIter; /**< maxIter is the number of RANSAC iterations*/
       bool useRANSAC; /**< useRANSAC set to true to use RANSAC*/
       double reprojThreshold; /**< reprojThreshold is the reprojection threshold in pixel*/
       double confidence; /**< confidence is the confidence to reach [0:1]*/
   };

   static Param getDefaultParam();

   Reconstruction3D();
   ~Reconstruction3D();
   int compute(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Point3d> &pts3D, cv::Mat &R, cv::Mat &t, cv::Mat &error, Param param = getDefaultParam());

// protected:
   int computeWith2Persp(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Point3d> &pts3D, cv::Mat &R, cv::Mat &t, cv::Mat &error, Param param = getDefaultParam());
   int computeWith2Omni(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Point3d> &pts3D, cv::Mat &R, cv::Mat &t, cv::Mat &error, Param param = getDefaultParam());
   int computeWithPerspAndOmni(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Point3d> &pts3D, cv::Mat &R, cv::Mat &t, cv::Mat &error, Param param = getDefaultParam());

// fundamental matrix
   int findFundamentalMat( std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &F, Param p = getDefaultParam());
   int findFundamentalMat( std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &F, cv::Mat &error, Param p = getDefaultParam());
   int findFundamentalMat( std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &F, bool useRANSAC = true, double reprojThreshold = 1.0, double confidence = 0.99, int maxIter = 2000);
   int findFundamentalMat( std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, cv::Mat &F, cv::Mat &error, bool useRANSAC = true, double reprojThreshold = 1.0, double confidence = 0.99, int maxIter = 2000);
   int findFundamentalMat( cv::Mat &pts1, cv::Mat &pts2, cv::Mat &F, cv::Mat &error, bool useRANSAC = true, double reprojThreshold = 1.0, double confidence = 0.99, int maxIter = 2000);

// essential matrix
   int computeEssentialMatrix(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, Camera &cam1, Camera &cam2, cv::Mat &E, cv::Mat &error, Param p = getDefaultParam());
   //int computeEssentialMatrix2(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat &K1, cv::Mat &K2, cv::Mat &E, double epsilon1 = 0, double epsilon2 = 0, bool useRANSAC = true);
   //int computeEssentialMatrix(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat &K1, cv::Mat &K2, cv::Mat &E, double epsilon1 = 0, double epsilon2 = 0, bool useRANSAC = true);
   int computeEssentialMatrix(cv::Mat &F, cv::Mat &K1, cv::Mat &K2, cv::Mat &E);
   int computeEssentialMatrix(cv::Mat &R, cv::Mat &t, cv::Mat &E);

// extrinsic parameters
   int extractExtrinsicParametersFromEssential(cv::Mat &E, cv::Mat &R, cv::Mat &t);
   int extractExtrinsicParametersFromEssential(cv::Mat &E, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec);
   int extractExtrinsicParametersFromFondamental(cv::Mat &F, cv::Mat &R, cv::Mat &t);

   int selectExtrinsicParameters(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera &cam1, Camera &cam2, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec, cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &pts3D, std::vector<bool> mask = std::vector<bool>());

   int computeExtrinsicParameters(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, Camera cam, bool useExtrinsicGuess = false);
   int computeExtrinsicParametersOpenCV(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, Camera cam, bool useExtrinsicGuess = false);
   int computeExtrinsicParametersWithSVD(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, cv::Mat K);
   int computeExtrinsicParametersWithLM(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, cv::Mat K);
   int computeExtrinsicParametersWithPseudoInverse(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, cv::Mat K);
   int computeExtrinsicParametersWithMatlabMethod(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, Camera cam);
   int computeExtrinsicParameters2(std::vector<cv::Point3d> &objectPoints, std::vector<cv::Point2d> &imagePoints, cv::Mat &R, cv::Mat &t, cv::Mat K);

// triangulation
   int triangulate(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, Camera cam1, Camera cam2, cv::Mat R, cv::Mat t, std::vector<cv::Point3d> &pts3D, cv::Mat R0 = cv::Mat::eye(3,3,CV_64F), cv::Mat t0 = cv::Mat::zeros(3,1,CV_64F));
   int triangulate(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat K1, cv::Mat K2, cv::Mat R, cv::Mat t, std::vector<cv::Point3d> &pts3D, cv::Mat R0 = cv::Mat::eye(3,3,CV_64F), cv::Mat t0 = cv::Mat::zeros(3,1,CV_64F));
   int triangulate(std::vector<cv::Point2d> &pts1n, std::vector<cv::Point2d> &pts2n, cv::Mat R, cv::Mat t, std::vector<cv::Point3d> &pts3D, cv::Mat R0 = cv::Mat::eye(3,3,CV_64F), cv::Mat t0 = cv::Mat::zeros(3,1,CV_64F));
   int triangulate(cv::Point2d pt1n, cv::Point2d pt2n, cv::Mat R, cv::Mat t, cv::Point3d &Q, cv::Mat R0 = cv::Mat::eye(3,3,CV_64F), cv::Mat t0 = cv::Mat::zeros(3,1,CV_64F));
   int triangulate(cv::Point2d pt1, cv::Point2d pt2, cv::Mat K1, cv::Mat K2, cv::Mat R, cv::Mat t, cv::Point3d &Q, cv::Mat R0 = cv::Mat::eye(3,3,CV_64F), cv::Mat t0 = cv::Mat::zeros(3,1,CV_64F));
   int triangulate(std::vector<cv::Point2d> &ptsp, std::vector<cv::Point2d> &ptsc, cv::Mat &Rp, cv::Mat &tp, cv::Mat &Rc, cv::Mat &tc, cv::Mat &Kp, cv::Mat &Bc, std::vector<cv::Point3d> &pts3D);

// bundle adjustment
   int bundleAdjustment(std::vector< std::vector<cv::Point2d> > &imagesPoints, std::vector<cv::Point3d> &points3D, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec, cv::Mat K, int max_iter = 30);
   int bundleAdjustment_bak(std::vector< std::vector<cv::Point2d> > &imagesPoints, std::vector<cv::Point3d> &points3D, std::vector<cv::Mat> Rvec, std::vector<cv::Mat> tvec, cv::Mat K);
   int bundleAdjustment2(std::vector< std::vector<cv::Point2d> > &imagesPoints, std::vector<cv::Point3d> &points3D, std::vector< std::vector<int> > visibility, std::vector<cv::Mat> &Rvec, std::vector<cv::Mat> &tvec, cv::Mat K, int max_iter = 30);

// trifocal tenser
   int findTrifocalTensor(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2, std::vector<cv::Point2d> pts3, cv::Mat &T1, cv::Mat &T2, cv::Mat &T3, cv::Mat &K);

// retroprojection
   double reprojError(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, std::vector<bool> mask = std::vector<bool>());
   double reprojError(cv::Point2d pt1, cv::Point2d pt2);
   std::vector<bool> getMask(cv::Mat error, double threshold);

   std::vector<double> getRetroprojError(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, std::vector<bool> mask = std::vector<bool>());
   std::vector<cv::Point2d> getRetroprojPoints(std::vector<cv::Point3d> pts3D, Camera cam, cv::Mat R, cv::Mat t);
   cv::Mat getRetropjImage(cv::Mat &img, std::vector<cv::Point2d> pts2D, std::vector<cv::Point3d> pts3D, Camera cam, cv::Mat R, cv::Mat t);
   cv::Mat getRetropjImage(cv::Mat &img, std::vector<cv::Point2d> pts2D, std::vector<cv::Point3d> pts3D, Camera cam, cv::Mat R, cv::Mat t, std::vector<bool> &good_matching);
   cv::Mat getRetropjImage(cv::Mat img, std::vector<cv::Point2d> pts_input, std::vector<cv::Point2d> pts_reproj, std::vector<bool> good_matching = std::vector<bool>());


   double getLambda(cv::Mat m, cv::Mat P, cv::Mat M);

   cv::Mat backprojectionOmni(double r, double x0, double y0);

// estimator
   Estimator* estimator; /**< Performs RANSAC and other mathematical operations*/

// projection
   cv::Point3d projectOnSphere(cv::Point2d pt, cv::Mat R, cv::Mat t, Camera cam);
   cv::Point3d projectOnSphere(cv::Point2d pt, double epsilon);
   cv::Mat projectOnSphere(cv::Mat pts, double epsilon);

// debug
   std::string getError(int error);
};

#endif //RECONSTRUCTION3D_H
