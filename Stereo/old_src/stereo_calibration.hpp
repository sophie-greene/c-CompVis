/**
* @brief Stereo Calibration Header
* @file stereo_calibration.hpp
* @date 26/04/2012
*
*/


#ifndef _STEREO_CALIBRATION_HPP_
#define _STEREO_CALIBRATION_HPP_


#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <utility>
#include <ctype.h>

#include "ofAppGlutWindow.h"
#include "ofMain.h"
#include "ofxGui.h"
#include "config.hpp"


/*
 * This class performs calibration on the cameras before they are used by the other application
 */
 
class StereoCalibration {
public:
	StereoCalibration();
	~StereoCalibration();

	void setup(GlobalConfig &config);
	
	bool addSingle(cv::Mat &cam, cv::Mat &board);
	bool addPair(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &board0, cv::Mat &board1);
		
	float calibrateSingle(CameraParameters &in);

	bool findTransform(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &trans);
	bool findEpilines(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &points, cv::Mat &lines);
	
	void clear();

protected:

	void runStandardCalibrationCheck(CameraParameters &in0, CameraParameters &in1, ExtrinsicParameters &ex);

	bool findChessboard(cv::Mat &cam0, std::vector<cv::Point2f> &corners, cv::Mat &board);

	void addToCalibration(bool right, vector<cv::Point2f> &corners);

	void runCameraCalibration (int cam);
	void runStandardCalibration(bool useintrinsics);
	
	
	void standardRectify();
	bool readStringList( const std::string& filename, std::vector<std::string>& l);

	// Shared Object Accessor

	struct SharedObj {
	
		SharedObj(GlobalConfig &config) : mConfig(config) {};
		float mSquareSize;

		cv::Rect *validRoi;
    
		std::vector< std::vector<cv::Point2f> > *imagePoints;
		std::vector< std::vector<cv::Point3f> > objectPoints;
   
		GlobalConfig &mConfig;
	};

	boost::shared_ptr<SharedObj> mObj;

	
};

#endif
