/**
* @brief Utilities header
* @file utils.hpp
* @date 26/04/2012
*
*/


#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <opencv2/opencv.hpp>
#include <stdlib.h>

#include "config.hpp"

template<class T> inline std::string toStringS9(const T& t) {
	std::ostringstream stream;
	stream << t;
	return stream.str();
}


template<class T> inline T fromStringS9(const std::string& s) {
	std::istringstream stream (s);
	T t;
	stream >> t;
	return t;
}

/*
 * OpenGL useful functions
 */
 
std::string getTextureParameters(GLuint id);
std::string convertInternalFormatToString(GLenum format);
std::string getRenderbufferParameters(GLuint id);


/*
 * OpenCV related functions for file saving and loading
 */

bool findChessboard(cv::Mat &cam0, std::vector<cv::Point2f> &corners, cv::Mat &board, cv::Size &size );

bool loadCameraParameters(std::string filename, CameraParameters &ip);

bool saveCameraParameters(std::string filename, CameraParameters &ip);



#endif
