#ifndef CAMERACONNECT_H
#define CAMERACONNECT_H
#include <QtGui>
class CameraConnect: public QObject
{
   Q_Object
public:
    CameraConnect(int);
    void setDeviceNumber(int);
    int getDeviceNumber();
private:
    int deviceNumber;
};
#endif // CAMERACONNECT_H
