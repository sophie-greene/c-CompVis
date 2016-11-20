/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 1 of the cookbook:  
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
#include <opencv2/highgui/highgui.hpp>

// function that creates and returns an image
cv::Mat function() {

	// create image
	cv::Mat ima(240,320,CV_8U,cv::Scalar(100));
	// return it
	return ima;
}

int main() {

	// create image
	cv::Mat image;
	// print image size
	std::cout << "size: " << image.size().height << " , " 
          << image.size().width << std::endl;
	// open image
	image=  cv::imread("img.jpg");
	// check if image has been successfully read
	if (!image.data) { 
Binary file ./PhDmed/code/OpenCV/Chapter 01/main2.cpp matches
