#ifndef UVCCAPTURE_H
#define UVCCAPTURE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <jpeglib.h>
#include <time.h>
#include <linux/videodev.h>
#include<iostream>
#include <vector>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>

using namespace cv;
using namespace std;

Mat capture (int device,   int width, int height ,int brightness  , int contrast , int saturation, int gain );


#endif // UVCCAPTURE_H

