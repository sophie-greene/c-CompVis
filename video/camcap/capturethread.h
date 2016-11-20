#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

#include <QObject>
#include<QThread>
#include<QtCore>
#include<QMutex>
#include <QImage>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
#include <QDebug>
#include <boost/lexical_cast.hpp>
using namespace cv;
using namespace std;
class CaptureThread : public QThread
{
    Q_OBJECT
public:
    CaptureThread(int,int,int);
    void run();
    void stop();
    int getHeight();
    int getWidth();
    int getDevice();
    bool isDeviceOpened;
    QImage getQimg();
    QImage putImage(const Mat& );

signals:
    void imageAquired(Mat);
public slots:

private:
    bool stopped;
    bool deviceOpen;
    VideoCapture cap;
    int width;
    int height;
    int device;
    Mat qimg;
};

#endif // CAPTURETHREAD_H
