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

    void stop();
    int getHeight();
    int getWidth();
    int getDevice();
    bool isDeviceOpened;
     bool isCalibrating;

protected:
    void run();

signals:
    void imageAquired(QImage,int);
public slots:

private:
    volatile bool stopped;
    bool deviceOpened;
    VideoCapture cap;
    int width;
    int height;
    int device;


};

#endif // CAPTURETHREAD_H
