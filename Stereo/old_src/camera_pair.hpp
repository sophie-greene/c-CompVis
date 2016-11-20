/**
* @brief Camera Pair Header
* @file camera_pair.hpp
* @date 26/04/2012
*
*/

#ifndef __CAMERA_PAIR__
#define __CAMERA_PAIR__

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <opencv/highgui.h>
#include <gpu/gpu.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include "uvc_camera.hpp"
#include "utils.hpp"
#include "config.hpp"


/*
 * Camera Pairs for reconstruction based on two cameras at a time
 */
 
class CameraPair {

public:
	
	CameraPair() {};
	void setup(GlobalConfig &g, boost::shared_ptr<UVCVideo> dev0, boost::shared_ptr<UVCVideo> dev1, std::string ex, std::string ho);
	void update();
	
	cv::Mat& getMat0() { return mObj->mMat0; };
	cv::Mat& getMat1() { return mObj->mMat1; };
	
	cv::Mat& getMat0Flip() { return mObj->mMat0Flip; };
	cv::Mat& getMat1Flip() { return mObj->mMat1Flip; };
	
	cv::Mat& getBoard0() { return mObj->mBoard0; };
	cv::Mat& getBoard1() { return mObj->mBoard1; };
		
	cv::Mat& getMatTrans();
	cv::Mat getMat0Trans();
	cv::Mat& getMatDisp() { return mObj->mDisp; };
	
	ExtrinsicParameters getEP() {return mObj->mEP;};
	
	//cv::Mat findEpilines();
	
	void saveHomography(std::string ho="none");
	
	void loadHomography(std::string ho);
	
	cv::Mat getCalibrationImage();
	
	void collectCalibrationImages( boost::function<void (ThreadStatus)> f);
	
	void generateHomography();
	
	void computeDisparity(cv::Mat &m0, cv::Mat &m1, cv::Mat &disp, DisparityMethod m);
	
	void reproject(cv::Mat &disp, cv::Mat &reprojection);

protected:

	void _collectCalibrationImages(boost::function<void (ThreadStatus)> f);
	
	bool addPair(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &board0, cv::Mat &board1);
	
	void computeExtrinsics();
	
	struct SharedObj {
	
		SharedObj(GlobalConfig &config) : mConfig(config) {};
		float mSquareSize;
    
		std::vector< std::vector<cv::Point2f> > *imagePoints;
		std::vector< std::vector<cv::Point3f> > objectPoints;
   
		GlobalConfig &mConfig;
	
		// UVC Capture class shared pointers
		boost::shared_ptr<UVCVideo>		mUVC0;
		boost::shared_ptr<UVCVideo>		mUVC1;
		
		// Files that hold the calibrations for this pair
		std::string mExtrinsicFile;
		std::string mHomographyFile;
		ExtrinsicParameters mEP;
		
		// Associated Major matrices
		
		cv::Mat mMat0, mMat1, mDisp, mReprojected, mHomography;
		cv::Mat mMat0Flip, mMat1Flip; 
		cv::Mat mMatWarp;
		cv::Rect *validRoi;
		
		
		cv::gpu::StereoBM_GPU mBM;
		cv::gpu::StereoBeliefPropagation mSBP;
		cv::gpu::StereoConstantSpaceBP mSCS;
		
		
		//Calibration
		cv::Mat mBoard0, mBoard1; // results
		double mCInterval;
		double mCLastCall;
		
		ThreadStatus mThreadStatus;
		boost::thread *pWorkerThread;
	};

	boost::shared_ptr<SharedObj> mObj;
	
	static int sSeed; // Used to create IDs for the threads. Really needs to be done better
	
	
};
#endif
