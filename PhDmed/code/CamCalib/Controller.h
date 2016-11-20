//Sophie M Greene 07/12/2012
//controls threads and cam intialiasation
#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "CaptureThread.h"
#include "Structures.h"

// Qt header files
#include <QtGui>
// OpenCV header files
#include <opencv/highgui.h>

class ImageBuffer;

class Controller : public QObject
{
    Q_OBJECT

public:
    Controller();
    ~Controller();
    ImageBuffer *imageBuffer;

    CaptureThread *captureThread;
    bool connectToCamera(int);
    void disconnectCamera();
    void stopCaptureThread();
    void deleteCaptureThread();
    void clearImageBuffer();
    void deleteImageBuffer();
private:
    int imageBufferSize;
};

#endif // CONTROLLER_H
