#ifndef FRAMECAPTURE_H
#define FRAMECAPTURE_H
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
class FrameCaptureThread: public QThread
{
    Q_OBJECT
public:
    FrameCaptureThread(int,int,int);
    void run();
    void stop();
    void terminate();
    int getHeight();
    int getWidth();
    int getDevice();
    bool isDeviceOpened;

signals:

public slots:

private:
    bool stopped;
    bool deviceOpen;
    VideoCapture cap;
    int width;
    int height;
    int device;
};

#endif // FRAMECAPTURE_H
