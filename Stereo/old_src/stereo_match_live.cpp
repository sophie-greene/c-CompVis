/**
* @brief Stereo Matching Body
* @file stereo_match_live.cpp
* @date 26/04/2012
*
*/


/*
 An attempt to live stereo match with the camera feeds
*/

// http://opencv.itseez.com/modules/gpu/doc/camera_calibration_and_3d_reconstruction.html
// http://stackoverflow.com/questions/5987285/what-is-an-acceptable-return-value-from-cvcalibratecamera

#include "stereo_match_live.hpp"

using namespace cv;
using namespace std;

StereoGPU::StereoGPU() {
		

}

void StereoGPU::setup(GlobalConfig &config) {

	mObj.reset(new SharedObj(config)); 

	// PCL Filtering
	
	/*mObj->pCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	
	int w = mObj->mConfig.camSize.width;
	int h = mObj->mConfig.camSize.height;
	
	mObj->pCloud->width  = w;
	mObj->pCloud->height = h;
	mObj->pCloud->points.resize (w * h);
	
	mObj->pCloudFiltered.reset(new pcl::PointCloud<pcl::PointXYZ>);
	mObj->pCloudFiltered->width  = w;
	mObj->pCloudFiltered->height = h;
	mObj->pCloudFiltered->points.resize (w * h);*/
}

StereoGPU::~StereoGPU() {

}


void StereoGPU::computeDisparity(cv::Mat &m0, cv::Mat &m1, cv::Mat &disp, DisparityMethod m) {
	
	gpu::GpuMat gpuGrey1, gpuGrey2;
		
	Mat leftGray(m0.size(),CV_8UC1);
	Mat rightGray(m1.size(),CV_8UC1);

	cvtColor(m0, leftGray, CV_RGB2GRAY);
    cvtColor(m1, rightGray, CV_RGB2GRAY);	
	
	gpuGrey1.upload(leftGray);
	gpuGrey2.upload(rightGray);
	
    gpu::GpuMat d_disp( m0.size(), CV_8UC1);
    
    
    switch (m){
		case DISPARITY_BM: {
			mObj->mBM.preset =  mObj->mConfig.sobel;
			mObj->mBM.ndisp = mObj->mConfig.stepSize;
			mObj->mBM.winSize = mObj->mConfig.blockSize;
			
			mObj->mBM(gpuGrey1, gpuGrey2, d_disp);
			break;
		}
		case DISPARITY_BP : {
			
			mObj->mSBP.ndisp = mObj->mConfig.disp;
			mObj->mSBP.iters =  mObj->mConfig.iters;
			mObj->mSBP.levels =  mObj->mConfig.levels;
			
			mObj->mSBP(gpuGrey1, gpuGrey2, d_disp);
			
			
			break;
		}
		case DISPARITY_CSBP : {
			
			mObj->mSCS.ndisp = mObj->mConfig.disp;
			mObj->mSCS.iters =  mObj->mConfig.iters;
			mObj->mSCS.nr_plane =  mObj->mConfig.near;
			mObj->mSCS.levels =  mObj->mConfig.levels;
			
			mObj->mSCS(gpuGrey1, gpuGrey2, d_disp);
			
			break;
		}
	}
    

	//mSBP(mGPULeft, mGPURight, disp); // Has memory issues apparently on my laptop
	
	d_disp.download(disp);
}

/*
void StereoGPU::reproject(cv::Mat &disp, cv::Mat &reprojection) {
	gpu::GpuMat gpuDisp, gpuFinal;
	gpuDisp.upload(disp);
	Mat q;
	mEP.Q.convertTo(q,CV_32F,1.0);
	gpu::reprojectImageTo3D(gpuDisp, gpuFinal, q);
	gpuFinal.download(reprojection);

}

void StereoGPU::cleanAndCreate(cv::Mat &reprojection) {
	
	int k = 0;
	for (int i = 0; i < mWidth; i++){
		for (int j = 0; j < mHeight; j ++){
			float z = reprojection.at<float>(j , (i * 4) + 2);
			pCloud->points[k].x = (float)i;
			pCloud->points[k].y = mHeight - (float)j;
			pCloud->points[k].z = z;
			k++;
		}   
	}
	
	mPass.setInputCloud (pCloud);
	mPass.setFilterFieldName ("z");
	mPass.setFilterLimits (-100.0,100.0);
	mPass.filter (*pCloudFiltered);
}  

void StereoGPU::createSurface() {
	
	if (pCloudFiltered->points.size() > 0){
		// Normal estimation*
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (pCloudFiltered);
		n.setInputCloud (pCloudFiltered);
		n.setSearchMethod (tree);
		n.setKSearch (20);
		n.compute (*normals);
		
		//* normals should not contain the point normals + surface curvatures

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*pCloudFiltered, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		

		// Set the maximum distance between connected points (maximum edge length)
		gp3.setSearchRadius (0.025);

		// Set typical values for the parameters
		gp3.setMu (2.5);
		gp3.setMaximumNearestNeighbors (100);
		gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
		gp3.setMinimumAngle(M_PI/18); // 10 degrees
		gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
		gp3.setNormalConsistency(false);

		// Get result
		gp3.setInputCloud (cloud_with_normals);
		gp3.setSearchMethod (tree2);
		gp3.reconstruct (mTriangles);

		// Additional vertex information
		std::vector<int> parts = gp3.getPartIDs();
		std::vector<int> states = gp3.getPointStates();
	}
}*/
