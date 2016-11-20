

#ifndef UVCCAPTURE_H
#define UVCCAPTURE_H
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
using namespace cv;
Mat capture (char device[],char outFile[],   int width, int height ,int brightness  , int contrast , int saturation, int gain );


#endif // UVCCAPTURE_H

