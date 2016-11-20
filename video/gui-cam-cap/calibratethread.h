#ifndef CALIBRATETHREAD_H
#define CALIBRATETHREAD_H
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
#include "cameracalibrator.h"
using namespace cv;
using namespace std;

class CalibrateThread: public QThread
{
    Q_OBJECT
public:
    CalibrateThread(int,int);
    void stop();
    int getWidth();
    int getHeight();

protected:
    void run();

signals:
    void calibrationSuccess(QImage, int);

private:
    volatile bool stopped;
    int width;
    int height;
     vector< string > fileList;
    void calibrate();

public slots:


};

#endif // CALIBRATETHREAD_H
