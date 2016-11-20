#ifndef CALIBRATETHREAD_H
#define CALIBRATETHREAD_H
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

using namespace cv;
using namespace std;

class CalibrateThread: public QThread
{
    Q_OBJECT
public:
    CalibrateThread(int,int,int);
    void stop();
    int getWidth();
    int getHeight();
protected:
    void run();

signals:
    void calibrationSuccess(QImage,int);

private:

    VideoCapture cap;
    volatile bool stopped;
    int device;
    int noOfFarmes;
    int width;
    int height;
    void calibrate(int,const vector<string>& ,vector<vector<Point2f> > & );
    void stereoCalib(int, const vector<string>&,int, const vector<string>&);
public slots:


};

#endif // CALIBRATETHREAD_H
