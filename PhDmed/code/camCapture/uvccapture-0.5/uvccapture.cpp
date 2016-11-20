
/*******************************************************************************
#             uvccapture: USB UVC Video Class Snapshot Software                #
#This package work with the Logitech UVC based webcams with the mjpeg feature  #
#.                                                                             #
# 	Orginally Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard   #
#       Modifications Copyright (C) 2006  Gabriel A. Devenyi                   #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

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
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
using namespace cv;
using namespace std;
#include "v4l2uvc.h"

//static const char version[] = "VERSION";
//int run = 1;



//Mat mainfu (char device[],char outFile[],   int width, int height ,int brightness  , int contrast , int saturation, int gain );
Mat capture (char device[],char outFile[],   int width, int height ,int brightness  , int contrast , int saturation, int gain )
{
    char *videodevice = device;
    char *outputfile = outFile;
    char *post_capture_command[3];
    int format = V4L2_PIX_FMT_MJPEG;
    int grabmethod = 1;

    //Mat frame;
    int verbose = 120;
    int delay = 0;
    int quality = 95;
    int post_capture_command_wait = 0;
    time_t ref_time;
    struct vdIn *videoIn;
    FILE *file;



    // set post_capture_command to default values
    post_capture_command[0] = NULL;
    post_capture_command[1] = NULL;
    post_capture_command[2] = NULL;


    if ((width > 960) || (height > 720) || (quality != 95))
        format = V4L2_PIX_FMT_YUYV;

    if (post_capture_command[0])
        post_capture_command[1] = outputfile;

    if (verbose >= 1) {
        fprintf (stderr, "Using videodevice: %s\n", videodevice);
        fprintf (stderr, "Saving images to: %s\n", outputfile);
        fprintf (stderr, "Image size: %dx%d\n", width, height);
        fprintf (stderr, "Taking snapshot every %d seconds\n", delay);
        if (grabmethod == 1)
            fprintf (stderr, "Taking images using mmap\n");
        else
            fprintf (stderr, "Taking images using read\n");
        if (post_capture_command[0])
            fprintf (stderr, "Executing '%s' after each image capture\n",
                     post_capture_command[0]);
    }
    videoIn = (struct vdIn *) calloc (1, sizeof (struct vdIn));
    if (init_videoIn
        (videoIn, (char *) videodevice, width, height, format, grabmethod) < 0)
        exit (1);

    //Reset all camera controls
    if (verbose >= 1)
        fprintf (stderr, "Resetting camera settings\n");
    v4l2ResetControl (videoIn, V4L2_CID_BRIGHTNESS);
    v4l2ResetControl (videoIn, V4L2_CID_CONTRAST);
    v4l2ResetControl (videoIn, V4L2_CID_SATURATION);
    v4l2ResetControl (videoIn, V4L2_CID_GAIN);

    //Setup Camera Parameters
    if (brightness != 0) {
        if (verbose >= 1)
            fprintf (stderr, "Setting camera brightness to %d\n", brightness);
        v4l2SetControl (videoIn, V4L2_CID_BRIGHTNESS, brightness);
    } else if (verbose >= 1) {
        fprintf (stderr, "Camera brightness level is %d\n",
                 v4l2GetControl (videoIn, V4L2_CID_BRIGHTNESS));
    }
    if (contrast != 0) {
        if (verbose >= 1)
            fprintf (stderr, "Setting camera contrast to %d\n", contrast);
        v4l2SetControl (videoIn, V4L2_CID_CONTRAST, contrast);
    } else if (verbose >= 1) {
        fprintf (stderr, "Camera contrast level is %d\n",
                 v4l2GetControl (videoIn, V4L2_CID_CONTRAST));
    }
    if (saturation != 0) {
        if (verbose >= 1)
            fprintf (stderr, "Setting camera saturation to %d\n", saturation);
        v4l2SetControl (videoIn, V4L2_CID_SATURATION, saturation);
    } else if (verbose >= 1) {
        fprintf (stderr, "Camera saturation level is %d\n",
                 v4l2GetControl (videoIn, V4L2_CID_SATURATION));
    }
    if (gain != 0) {
        if (verbose >= 1)
            fprintf (stderr, "Setting camera gain to %d\n", gain);
        v4l2SetControl (videoIn, V4L2_CID_GAIN, gain);
    } else if (verbose >= 1) {
        fprintf (stderr, "Camera gain level is %d\n",
                 v4l2GetControl (videoIn, V4L2_CID_GAIN));
    }
    ref_time = time (NULL);
    int i=0;
    // while (1) {
   if (verbose >= 2)
       fprintf (stderr, "Grabbing frame\n");
    if (uvcGrab (videoIn) < 0) {
        fprintf (stderr, "Error grabbing\n");
        close_v4l2 (videoIn);
        free (videoIn);
        exit (1);
    }


       if (verbose >= 1)
          fprintf (stderr, "Saving image to: %s\n", outputfile);
        file = fopen (outputfile, "wb");



            fwrite (videoIn->tmpbuffer, videoIn->buf.bytesused + DHT_SIZE, 1,
                    file);
           // cout<<"std:"<<&videoIn->tmpbuffer;
           //printf ( "--->%d",videoIn->buf.bytesused);
             Mat image(Size(videoIn->width, videoIn->height), CV_8UC1, videoIn->tmpbuffer, Mat::AUTO_STEP);
            // imshow("canny",image);
            // Mat image(Size(width, height), CV_8UC1, dataBuffer, Mat::AUTO_STEP);
          fclose (file);
            videoIn->getPict = 0;

        if (post_capture_command[0]) {
            if (verbose >= 1)
                fprintf (stderr, "Executing '%s %s'\n", post_capture_command[0],
                         post_capture_command[1]);

        }

    // if (delay == 0)
    // break;
    // }
    close_v4l2 (videoIn);
    free (videoIn);

    return image;
}
