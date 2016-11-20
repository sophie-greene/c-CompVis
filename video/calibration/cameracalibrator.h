#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H
#include<opencv/cv.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"


#include<QtCore>


#include "opencv2/opencv.hpp"
#include <QDebug>
#include <boost/lexical_cast.hpp>

using namespace cv;
using namespace std;

class CameraCalibrator {

    // input points
    vector<vector<Point3f> > objectPoints;
    vector<vector<Point2f> > imagePoints;
    // output Matrices
    Mat cameraMatrix;
    Mat distCoeffs;
    Mat rotationMatrix;
    Mat translationVector;
    // flag to specify how calibration is done
    int flag;
    // used in image undistortion
    Mat map1,map2;
    bool mustInitUndistort;

public:
    CameraCalibrator() : flag(0), mustInitUndistort(true) {};
    Mat convertVector3fToMat(const vector<vector<Point3f> > &);
    Mat convertVector2fToMat(const vector<vector<Point2f> > &);
    // Open the chessboard images and extract corner points
    int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);
    // Add scene points and corresponding image points
    void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
    // Calibrate the camera
    double calibrate(cv::Size &imageSize);
    // Set the calibration flag
    void setCalibrationFlag(bool radial8CoeffEnabled=false, bool tangentialParamEnabled=false);
    // Remove distortion in an image (after calibration)
    cv::Mat remap( const cv::Mat &image);

    // Getters
    cv::Mat getCameraMatrix() { return cameraMatrix; }
    cv::Mat getDistCoeffs()   { return distCoeffs; }
    cv::Mat getRotationMatrix(){return rotationMatrix;}
    cv::Mat getTranslationVector(){return translationVector;}
};
#endif // CAMERACALIBRATOR_H
