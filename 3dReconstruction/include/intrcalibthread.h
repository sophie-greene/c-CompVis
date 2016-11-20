#ifndef INTRCALIBTHREAD_H
#define INTRCALIBTHREAD_H
#include <QObject>
#include<QThread>
#include<QtCore>
#include<QMutex>
#include <QImage>
#include<core/core.hpp>
#include<highgui/highgui.hpp>
#include<cv.h>
#include <opencv.hpp>
#include <QDebug>
#include <boost/lexical_cast.hpp>
#include "cameracalibrator.h"
#include "include/uvccapture.h"

class IntrCalibThread : public QThread
{
    Q_OBJECT
public:
    IntrCalibThread(int,int,int,bool);
    void stop();
    void readCamPar();
    void readMat(const string& filename, Mat & mat1,Mat & mat2,const string & str1,const string &str2);
    Mat combineMat(const Mat & mat1,const Mat & mat2,int col,int row);
    bool addChessboardPoints(const Mat& img, Size & boardSize,Mat&);
    // Add scene points and corresponding image points
    void addPoints(const vector<Point2f>& imageCorners, const vector<Point3f>& objectCorners);
    bool isStopped();
    void displayMatrix(Mat& );
    Mat remap(const Mat &image);
    bool storeMatrices();
    void computeWorld();
protected:
    void run();

signals:
    void intrCalibrationSuccess(QImage,int);
    void intrCalibStop(int,bool);

private:
    VideoCapture cap;
    volatile bool stopped;
    int device;
    int width;
    int height;
    void calibrate(int,const vector<string>& ,vector<vector<Point2f> > & );
    // used in image undistortion
    Mat map1,map2;
    bool mustInitUndistort;
    bool useIntrinsics;
    Mat cameraMatrix,distCoeffs;
     Mat rotationMatrix,translationVector;
    // input points
    vector<vector<Point3f> > objectPoints;
    vector<vector<Point2f> > imagePoints;
    Point2f world[8][4];//world is 4 element matrix where world[0]is orginig, world[1] is Nx, world[2] is Ny, world[3] is Nz


public slots:


};

#endif // INTRCALIBTHREAD_H
