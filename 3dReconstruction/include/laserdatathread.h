#ifndef LASERDATATHREAD_H
#define LASERDATATHREAD_H
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
#include "captureframethread.h"
#include "uvccapture.h"
using namespace cv;
using namespace std;
class LaserDataThread : public QThread
{
    Q_OBJECT
public:
    LaserDataThread(int,int);

    void stop();
    int getHeight();
    int getWidth();
    int getDevice();
    bool isDeviceOpened;
void writePoint(const vector<Point> & pts);
protected:
    void run();

signals:
    void LaserAquis( QImage,int);
public slots:

private:
    volatile bool stopped;
    bool deviceOpened;
    VideoCapture cap[8];
    int width;
    int height;
    int threadCnt;
    //vector<QImage> imgSet;
   // vector<Point2f> imgPoints; //8x2 matrix containing points frame all cameras
   void detectPoint(const Mat&,int & x,int & y);
};


#endif // LASERDATATHREAD_H
