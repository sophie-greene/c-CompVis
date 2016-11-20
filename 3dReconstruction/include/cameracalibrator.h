#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <iostream>
#include <iomanip>
#include <vector>
#include<QDebug>
#include <core/core.hpp>
#include <imgproc/imgproc.hpp>
#include <highgui/highgui.hpp>
#include <features2d/features2d.hpp>
#include <calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

class CameraCalibrator {
private:
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
    int addChessboardPoints(const vector<string>& filelist, Size & boardSize,vector<string>& successflst);
    // Add scene points and corresponding image points
    void addPoints(const vector<Point2f>& imageCorners, const vector<Point3f>& objectCorners);
    // Calibrate the camera
    double calibrateI(Size &imageSize);
    double calibrateE(Size &imageSize);
    //stereo calibration
    double stereoCal(vector<vector<Point3f> > & objectPointsRef,
                    vector<vector<Point2f> > &imagePointsRef,
                    Mat &cameraMatrixRef,
                    Mat &distCoeffsRef,
                    Size &imageSize,
                    Mat R, Mat T,
                    Mat E,
                    Mat F);

    // Set the calibration flag
    void setCalibrationFlag(bool radial8CoeffEnabled=false, bool tangentialParamEnabled=false);
    // Remove distortion in an image (after calibration)
    Mat remap( const Mat &image);

    // Getters
    Mat getCameraMatrix() { return cameraMatrix; }
    Mat getDistCoeffs()   { return distCoeffs; }
    Mat getRotationMatrix(){return rotationMatrix;}
    Mat getTranslationVector(){return translationVector;}
    vector<vector<Point3f> > getObjectPoints(){return objectPoints;}
    vector<vector<Point2f> > getImagePoints(){return imagePoints;}
};
#endif // CAMERACALIBRATOR_H
