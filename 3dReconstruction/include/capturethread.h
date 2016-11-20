#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

#include <QObject>
#include<QThread>
#include<QtCore>
#include<QMutex>
#include <QImage>
#include<core/core.hpp>
#include<highgui/highgui.hpp>
#include<cv.h>
#include <QDebug>
#include <boost/lexical_cast.hpp>
#include "uvccapture.h"
using namespace cv;
using namespace std;
class CaptureThread : public QThread
{
    Q_OBJECT
public:
    CaptureThread(int,int,int);

    void stop();
    int getHeight();
    int getWidth();
    int getDevice();
    bool isDeviceOpened;
    void readMat(const string& filename, Mat & mat1,Mat & mat2,const string & str1,const string &str2);
    Mat combineMat(const Mat & mat1,const Mat & mat2,int col,int row);
    void readCamPar();
    bool storeMatrices();
    void computeWorld();
   void displayMatrix(const Mat&);
protected:
    void run();

signals:
    void imageAquired(QImage,int,int,int);
    void laserAquired(int,int,int,int);
public slots:

private:
    volatile bool stopped;
    bool deviceOpened;
    VideoCapture cap;
    int width;
    int height;
    int device;
    int threadCnt;
    void detectPoint(const Mat & ,int&,int&);
    Mat rotationMatrix;
    Mat distCoeffs;
    Mat translationVector;
    Mat cameraMatrix;
    Point2f world[8][4];//world is 4 element matrix where world[0]is orginig, world[1] is Nx, world[2] is Ny, world[3] is Nz

};

#endif // CAPTURETHREAD_H
