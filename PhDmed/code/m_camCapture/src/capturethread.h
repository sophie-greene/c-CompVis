#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

#include <QObject>
#include<QThread>
#include<QtCore>
#include<QMutex>
#include <QImage>
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
    QImage getQimg();

signals:
    void imageAquired(bool);
public slots:

private:
    bool stopped;
    int width;
    int height;
    int device;
    QImage qimg;
};

#endif // CAPTURETHREAD_H
