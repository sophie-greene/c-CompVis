#ifndef CAPTUREFRAMETHREAD_H
#define CAPTUREFRAMETHREAD_H
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
using namespace cv;
using namespace std;
class CaptureFrameThread : public QThread
{
    Q_OBJECT
public:
    CaptureFrameThread(int,int,int);

    void stop();
    int getHeight();
    int getWidth();
    int getDevice();
    bool isDeviceOpened;

protected:
    void run();

signals:
    void imageAquired(Mat,int);
public slots:

private:
    volatile bool stopped;
    bool deviceOpened;
    VideoCapture cap;
    int width;
    int height;
    int device;
};

#endif // CAPTUREFRAMETHREAD_H
