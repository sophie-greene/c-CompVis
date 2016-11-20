/**
* @brief Epipolar Method Class Header
* @file epipolar.hpp
* @date 03/05/2012
*
*/

#ifndef _EPIPOLAR_HPP_
#define _EPIPOLAR_HPP_

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <boost/shared_ptr.hpp>

#include <opencv/highgui.h>
#include <gpu/gpu.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include "uvc_camera.hpp"
#include "utils.hpp"
#include "config.hpp"


class Epipolar {
public:
	Epipolar() {};

	void setup(GlobalConfig &config);
	void stop() {mObj->mRunning = false; };
	cv::Mat& getEpiMatrix(int i);
	
	void startComputeEpilines(std::vector< boost::shared_ptr<UVCVideo> > &cameras);

protected:

	void computeEpilines(std::vector<boost::shared_ptr<UVCVideo> > &cameras);
	bool findEpilines(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &points, cv::Mat &lines);


	struct SharedObj {
	
		SharedObj(GlobalConfig &config) : mConfig(config) {};
		
		boost::thread *pWorkerThread;
		int mProgress;
		bool mRunning;
		GlobalConfig &mConfig;
		std::vector<cv::Mat> vMats;
	};

	boost::shared_ptr<SharedObj> mObj;

};


#endif
