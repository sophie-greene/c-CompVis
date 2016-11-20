/**
* @brief Epipolar Method Class Body
* @file epipolar.cpp
* @date 03/05/2012
*
*/

#include "epipolar.hpp"


using namespace std;
using namespace cv;
using namespace boost;


void Epipolar::setup(GlobalConfig &config) {
	mObj.reset(new SharedObj(config)); 
}

void Epipolar::startComputeEpilines(vector<shared_ptr<UVCVideo> > &cameras) {
	mObj->vMats.clear();
	 
	for (vector<shared_ptr<UVCVideo> >::iterator it = cameras.begin(); it != cameras.end(); it++){
		 mObj->vMats.push_back( Mat( mObj->mConfig.camSize, CV_8UC3)  );
	}
	
	mObj->mRunning = true;
	mObj->pWorkerThread = new boost::thread(&Epipolar::computeEpilines, this, cameras);
}

/*
 * This thread grabs images from the cameras and then calls compute epiplines on the collected images
 * Images are not rotated this time!
 */

void Epipolar::computeEpilines(vector<shared_ptr<UVCVideo> > &cameras) {
	
	mObj->mProgress = 1;

	while (mObj->mRunning) {
		
		mObj->vMats[0] = cameras[0]->getBufferCorrected().clone(); // Take first camera as master

		while (mObj->mProgress < cameras.size()) {
			
			Mat cm = cameras[mObj->mProgress]->getBufferCorrected().clone();
			mObj->vMats[mObj->mProgress] = cm.clone();
			Mat lines;
					
			//if (findEpilines(mObj->vMats[0], cm, mObj->vMats[mObj->mProgress], lines)) {
				// Set the corresponding matrix to be the same as points
			
			findEpilines(mObj->vMats[0], cm, mObj->vMats[mObj->mProgress], lines);

			mObj->mProgress++;
			
		}
	
		mObj->mProgress = 1;
	}
}

bool Epipolar::findEpilines(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &points, cv::Mat &lines){
	vector<Point2f> corners0;
	vector<Point2f> corners1;
	
	Mat board (mObj->mConfig.camSize, CV_8UC3);

	if (findChessboard(cam0,corners0,board,mObj->mConfig)){
		if (findChessboard(cam1,corners1,board,mObj->mConfig)){
			Mat F =  findFundamentalMat(corners0, corners1, CV_FM_8POINT);
			computeCorrespondEpilines(corners0, 0, F, lines);
			
			// Draw the epilines on points
			
		//	points = cam1.clone();
		
			for (int i=0; i < 1 /*lines.rows*/; i ++){
				float a = lines.ptr<float>(i)[0];
				float b = lines.ptr<float>(i)[1];
				float c = lines.ptr<float>(i)[2];
				
				//ax + by + c = 0 -> by = -ax - c  y = (-ax - c) / b;
				// Take x to be 0 and width
				
				cv::Point2f p0 = cv::Point2f(0.0, -c / b);
				cv::Point2f p1 = cv::Point2f(points.size().width, (-points.size().width * a - c) / b );
				
				cv::line(points, p0, p1, cv::Scalar(215, 29, 35));
			}
					
			return true;
		}
	}
	return false;
}

/*
 * When asked, return all the matrices
 */

cv::Mat& Epipolar::getEpiMatrix(int i) {
	return mObj->vMats[i];
}
