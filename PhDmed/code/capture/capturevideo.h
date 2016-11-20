#ifndef CAPTUREVIDEO_H
#define CAPTUREVIDEO_H
#include "v4l2uvc.h"
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
#include <linux/videodev.h>
using namespace cv;
class CaptureVideo
{
public:
    CaptureVideo();
    Mat getFrame();
    bool startCapture (int,int,int,int,int,int,int );
    void endCapture();
private:
    struct vdIn *videoIn;
    int device;
    int width;
    int heigh;
    int brightness;
    int contrast;
    int saturation;
    int gain;
};

#endif // CAPTUREVIDEO_H
