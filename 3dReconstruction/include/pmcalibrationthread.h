#ifndef PMCALIBRATIONTHREAD_H
#define PMCALIBRATIONTHREAD_H
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

#define PM_IMAGES_PATH "photometricCalibrationImages/angle"
using namespace cv;
using namespace std;
class PhotoMCalibrationThread: public QThread
{
    Q_OBJECT
public:
    PhotoMCalibrationThread();
    void stop();
protected:
    void run();

signals:
   void pmcalibrationSuccess(bool);

private:
    volatile bool stopped;
    void pmCalibrate(int ,const vector<string>& );

public slots:


};

#endif // PMCALIBRATIONTHREAD_H
