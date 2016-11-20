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
    CaptureThread(int,int);
    void stop();
    void run();
    bool stopped;
    int getHeight();
    int getWidth();
     QImage qimg[8];
    QImage getQimg(int);

signals:
    void imageAquired(bool);
public slots:

private:

    int width;
    int height;


};

#endif // CAPTURETHREAD_H
