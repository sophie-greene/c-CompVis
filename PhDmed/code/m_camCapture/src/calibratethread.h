#ifndef CALIBRATETHREAD_H
#define CALIBRATETHREAD_H
#include <QObject>
#include<QThread>
#include<QtCore>
#include<QMutex>
#include <QImage>
#include <boost/lexical_cast.hpp>
#include "src/uvccapture.h"

class CalibrateThread: public QThread
{
    Q_OBJECT
public:
    CalibrateThread();
    void run();
    void stop();

signals:
    void calibrationSuccess(bool);
private:
    bool stopped;

};

#endif // CALIBRATETHREAD_H
