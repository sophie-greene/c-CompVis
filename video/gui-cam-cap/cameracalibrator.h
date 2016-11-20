#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <iostream>
#include <iomanip>
#include <vector>
#include<QDebug>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;
class CameraCalibrator {

    // input points
    vector<vector<Point3f> > objectPoints;
    vector<vector<Point2f> > imagePoints;
    // output Matrices
    Mat cameraMatrix;
    Mat distCoeffs;
    // flag to specify how calibration is done
    int flag;
    // used in image undistortion
    Mat map1,map2;
    bool mustInitUndistort;

  public:
    CameraCalibrator() : flag(0), mustInitUndistort(true) {};

    // Open the chessboard images and extract corner points
    int addChessboardPoints(const vector<string>& filelist, Size & boardSize);
    // Add scene points and corresponding image points
    void addPoints(const vector<Point2f>& imageCorners, const vector<Point3f>& objectCorners);
    // Calibrate the camera
    double calibrate(Size &imageSize);
    // Set the calibration flag
    void setCalibrationFlag(bool radial8CoeffEnabled=false, bool tangentialParamEnabled=false);
    // Remove distortion in an image (after calibration)
    Mat remap( const Mat &image);

    // Getters
    Mat getCameraMatrix() { return cameraMatrix; }
    Mat getDistCoeffs()   { return distCoeffs; }
};
#endif // CAMERACALIBRATOR_H
