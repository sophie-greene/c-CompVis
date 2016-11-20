/**
* @brief Global Configuration parameters and headers
* @file config.hpp
* @date 26/04/2012
*
*/

#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_

#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <string>
#include <vector>
#include <queue>
#include <fstream>


#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/ptr_container/ptr_deque.hpp>
#include <boost/ptr_container/ptr_list.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>


#include <GL/glew.h>
#include <GL/glut.h>

// Optional Headers depening on App

#include <opencv2/opencv.hpp>


typedef struct {
	// OpenCV Camera and calibration pairs info
	cv::Size boardSize;
	cv::Size camSize;
	int fps;
	float squareSize;
	int startCam;
	int endCam;
	float interval;
	int maxImages;
	
	// Point Detection Parameters
	double_t pointThreshold;
	double_t scanInterval;
	
	// World Sizes
	float xs,ys,zs;
	float xe,ye,ze;
	
	cv::Point3i meshResolution;
	
	// PCL Values
	int poissonDepth;
	float poissonSamples;
	float poissonScale;
	
	float pclSearchRadius;
	float pclPolynomialOrder;
	float pclUpsamplingRadius;
	float pclUpsamplingStepSize;
	
	float pclFilterMeanK;
	float pclFilterThresh;


}GlobalConfig;


inline bool checkError(int line) {
	int Error;
	if((Error = glGetError()) != GL_NO_ERROR){
		std::string ErrorString;
		switch(Error)
		{
		case GL_INVALID_ENUM:
			ErrorString = "GL_INVALID_ENUM";
			break;
		case GL_INVALID_VALUE:
			ErrorString = "GL_INVALID_VALUE";
			break;
		case GL_INVALID_OPERATION:
			ErrorString = "GL_INVALID_OPERATION";
			break;
		case GL_INVALID_FRAMEBUFFER_OPERATION:
			ErrorString = "GL_INVALID_FRAMEBUFFER_OPERATION";
			break;
		case GL_OUT_OF_MEMORY:
			ErrorString = "GL_OUT_OF_MEMORY";
			break;
		default:
			ErrorString = "UNKNOWN";
			break;
		}
		std::cerr << "Leeds - OpenGL Error " << ErrorString << " at " << line << std::endl;
	}
	return Error == GL_NO_ERROR;
}


/*
 * VBO Data class to hold our primitives
 */

#define VBO_VERT 0x01
#define VBO_IDCE 0x02
#define VBO_COLR 0x04
#define VBO_NORM 0x08
#define VBO_TEXC 0x10
#define VBO_TEXI 0x20
 
class VBOData {
public:
	
	VBOData();
	~VBOData();
	
	void compile(size_t buffers);
	
	void bind();
	void unbind();
	
	void allocateColours();
	void allocateVertices();
	void allocateTexCoords();
	void allocateNormals();
	void allocateIndices();
	void allocateTexIDs();
	
	size_t mUsed;
	size_t mNumBufs;
	size_t mVID;
	size_t mIID;
	size_t mCID;
	size_t mNID;
	size_t mTID;
	size_t mTTD;
	
	GLuint mNumElements;				// May be smaller than the size of the arrays due to buffering
	GLuint mNumIndices;					// Different to the above remmeber
	
	GLuint vao, *vbo;
	std::vector<GLfloat> mVertices;		// Vertices as 3 floats
	std::vector<GLuint> mIndices; 		// Indicies into the above array. Multiply by 3
	std::vector<GLfloat> mTexCoords;	// texture coordinates in u,v or pixel coords
	std::vector<GLfloat> mNormals;		// normals as 3 floats
	std::vector<GLfloat> mColours;		// colours as 3 or 4 floats
	std::vector<GLuint> vTexIDs;		// texids as indicies to textures per vertex - samplers basically
	
}; 


/*
 * OpenCV parameters per camera - these hold intrinsic values and extrinsic-to-world values
 */

class CameraParameters{
public:
	
	CameraParameters() { M = cv::Mat::eye(3, 3, CV_64F); mCalibrated = false;};
	cv::Mat M, D, R, T; 		// Camera Matrix, Distortion co-efficients, rotation and translation matrices
	std::vector<cv::Mat> Rs;	// Rotations for each view
	std::vector<cv::Mat> Ts;	// Translations for each view
	bool mCalibrated;			// Is this calibrated and distortion free?
};


/*
 * Camera UVC Controls for the Logitech C910s
 */

typedef enum {
		BRIGHTNESS = 0x00980900,
		CONTRAST = 0x00980901,
		SATURATION = 0x00980902,
		GAIN = 0x00980913,
		SHARPNESS = 0x0098091b,
		AUTO_EXPOSURE = 0x009a0901,
		EXPOSURE = 0x009a0902,
		FOCUS = 0x009a090a,
		AUTO_FOCUS = 0x009a090c
}CameraControl;


#endif
