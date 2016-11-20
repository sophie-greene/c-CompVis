/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 10 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "videoprocessor.h"
using namespace cv;
using namespace std;
void draw(cv::Mat& img, cv::Mat& out) {

	img.copyTo(out);
	cv::circle(out, cv::Point(100,100),5,cv::Scalar(255,0,0),2);
}

void canny(cv::Mat& img, cv::Mat& out) {

	// Convert to gray
	cv::cvtColor(img,out,CV_BGR2GRAY);
	// Compute Canny edges
	cv::Canny(out,out,100,200);
	// Invert the image
	cv::threshold(out,out,128,255,cv::THRESH_BINARY_INV);
}

int main()
{

    VideoCapture cap(0); // open the video camera no. 0

       if (!cap.isOpened())  // if not success, exit program
       {
           cout << "ERROR: Cannot open the video file" << endl;
           return -1;
       }

    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

      double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
      double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

      cout << "Frame Size = " << dWidth << "x" << dHeight << endl;

      Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));

    VideoWriter oVideoWriter ("MyVideo.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true); //initialize the VideoWriter object

      if ( !oVideoWriter.isOpened() ) //if not initialize the VideoWriter successfully, exit the program
      {
           cout << "ERROR: Failed to write the video" << endl;
           return -1;
      }

       while (1)
       {

           Mat frame;

           bool bSuccess = cap.read(frame); // read a new frame from video

           if (!bSuccess) //if not success, break loop
          {
                cout << "ERROR: Cannot read a frame from video file" << endl;
                break;
           }

            oVideoWriter.write(frame); //writer the frame into the file

           imshow("MyVideo", frame); //show the frame in "MyVideo" window

           if (waitKey(10) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
          {
               cout << "esc key is pressed by user" << endl;
               break;
          }
       }

       return 0;


}
