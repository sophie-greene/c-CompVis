#ifndef CAPTURE_H
#define CAPTURE_H

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
#include "v4l2uvc.h"
using namespace cv;
class Capture
{
private:
    int width;
    int height;
    int gain;
    int brightness;
    int contrast;
    int saturation;
    struct vdIn *videoIn;
public:
    Capture();

    bool openCap();
    bool closeCap();
    Mat captureFrame();

    int getWidth();
    int getHeight();
    int getGain();
    int getBrightness();
    int getContrast();
    int getSaturation();

    void setWidth(int);
    void setHeight(int);
    void setGain(int);
    void setBrightness(int);
    void setContrast(int);
    void setSaturation(int);

};

#endif // CAPTURE_H
