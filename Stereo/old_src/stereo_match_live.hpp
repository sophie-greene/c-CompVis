/**
* @brief Stereo Matching Header
* @file stereo_match_live.hpp
* @date 26/04/2012
*
*/

#ifndef _STEREO_MATCH_HPP_
#define _STEREO_MATCH_HPP_

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

/*
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
*/


#include <opencv/highgui.h>
#include <gpu/gpu.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include "ofMain.h"
#include "stereo_calibration.hpp"
#include "utils.hpp"
#include "config.hpp"

typedef enum {
	DISPARITY_BM = 0,
	DISPARITY_BP = 1,
	DISPARITY_CSBP = 2
}DisparityMethod;


class StereoGPU {
public:
	StereoGPU();
	~StereoGPU();
	
	void setup(GlobalConfig &config);
	
	/*
	
	void setCalibration(CameraParameters in0, CameraParameters in1, ExtrinsicParameters ex);
	
	void setParams(bool sgbm, int disp, int iters, int near, int levels, bool sobel, int blocksize);*/
	
	void computeDisparity(cv::Mat &m0, cv::Mat &m1, cv::Mat &disp, DisparityMethod m);
	
	/*void reproject(cv::Mat &disp,cv::Mat &reprojection);
	void cleanAndCreate(cv::Mat &reprojection);
	void createSurface();
	
	void allocate(int w, int h);*/

protected:

	struct SharedObj {
	
		SharedObj(GlobalConfig &config) : mConfig(config) {};

		// PCL 
	/*
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered;
		pcl::PassThrough<pcl::PointXYZ> mPass;
	
		pcl::PolygonMesh mTriangles;*/
	
		// GPU Stereo Matching

		cv::gpu::StereoBM_GPU mBM;
		cv::gpu::StereoBeliefPropagation mSBP;
		cv::gpu::StereoConstantSpaceBP mSCS;
    
		// GPU Params
    
		bool mSGBM;
		int mDisp;
		int mIters;
		int mNear;
		int mLevels;
		bool mSobel;
		int mBlockSize;
  
		GlobalConfig &mConfig;
	};

	boost::shared_ptr<SharedObj> mObj;

};

#endif
