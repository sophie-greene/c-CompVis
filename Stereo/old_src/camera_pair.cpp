/**
* @brief Camera Pair Body
* @file camera_pair.cpp
* @date 26/04/2012
*
*/

#include "camera_pair.hpp"

using namespace std;
using namespace cv;
using namespace boost;

int CameraPair::sSeed = 0;

/// Leeds Camera Pairs

void CameraPair::setup(GlobalConfig &g, shared_ptr<UVCVideo> dev0, shared_ptr<UVCVideo> dev1, std::string ex="none", std::string ho="none") {
	
	mObj.reset(new SharedObj(g));
	
	// Allocations
	mObj->mMat0.create(Size(mObj->mConfig.camSize.width,mObj->mConfig.camSize.height),CV_8UC3);
	mObj->mMat1.create(Size(mObj->mConfig.camSize.width,mObj->mConfig.camSize.height),CV_8UC3);
	
	// Rotated ones
	mObj->mMat0Flip.create(Size(mObj->mConfig.camSize.height,mObj->mConfig.camSize.width),CV_8UC3);
	mObj->mMat1Flip.create(Size(mObj->mConfig.camSize.height,mObj->mConfig.camSize.width),CV_8UC3);
	mObj->mDisp.create(Size(mObj->mConfig.camSize.height,mObj->mConfig.camSize.width),CV_8UC1);
	
	
	// Rotated dependent on mode (probably need to clear this methinx?)
	if (mObj->mConfig.portrait) {
		mObj->mBoard0.create(Size(mObj->mConfig.camSize.height,mObj->mConfig.camSize.width),CV_8UC3);
		mObj->mBoard1.create(Size(mObj->mConfig.camSize.height,mObj->mConfig.camSize.width),CV_8UC3);
	} else {
		mObj->mBoard0.create(Size(mObj->mConfig.camSize.width,mObj->mConfig.camSize.height),CV_8UC3);
		mObj->mBoard1.create(Size(mObj->mConfig.camSize.width,mObj->mConfig.camSize.height),CV_8UC3);
	}
	
	//Double ones
	if (mObj->mConfig.portrait)
		mObj->mMatWarp.create(Size(mObj->mConfig.camSize.height * 2, mObj->mConfig.camSize.width),CV_8UC3);
	else
		mObj->mMatWarp.create(Size(mObj->mConfig.camSize.width, mObj->mConfig.camSize.height * 2),CV_8UC3);
		
	mObj->mReprojected.create(Size(mObj->mConfig.camSize.width,mObj->mConfig.camSize.height),CV_8UC3);
	
	mObj->mUVC0 = dev0;
	mObj->mUVC1 = dev1;

	mObj->mExtrinsicFile = ex;
	mObj->mHomographyFile = ho;
	
	mObj->mHomography = Mat::eye(3, 3, CV_64F);
	
	// Attempt to load initial variables
	loadExtrinsics(mObj->mExtrinsicFile, mObj->mEP);
	loadHomography(mObj->mHomographyFile);
	
	mObj->mThreadStatus.id = sSeed;
	mObj->mThreadStatus.state = THREAD_START;
	mObj->mThreadStatus.progress = 0;
	
	mObj->imagePoints = new vector< vector<Point2f> > [2];
	mObj->mSquareSize = 1.0f;	
	
	mObj->validRoi = new Rect[2];
	
	// +1 to seed
	sSeed++;
}

void CameraPair::update() {
	
	mObj->mMat0 = cv::Mat (mObj->mUVC0->getBufferCorrected());
	mObj->mMat1 = cv::Mat (mObj->mUVC1->getBufferCorrected());
		
	// Rotate and copy	
    transpose(mObj->mMat0, mObj->mMat0Flip);
    flip(mObj->mMat0Flip,mObj->mMat0Flip,0);
    
    transpose(mObj->mMat1, mObj->mMat1Flip);
    flip(mObj->mMat1Flip,mObj->mMat1Flip,1);

}


cv::Mat&  CameraPair::getMatTrans() {
	
	Mat warped (mObj->mMat0Flip.size(), CV_8UC3);

	Mat imageROI = mObj->mMatWarp(Rect(0, 0, mObj->mMatWarp.size().width / 2, mObj->mMatWarp.size().height));
	
	warpPerspective(mObj->mMat0Flip, warped, mObj->mHomography, mObj->mMat0Flip.size());
	
	warped.copyTo(imageROI);

	imageROI = mObj->mMatWarp(Rect(mObj->mMatWarp.size().width / 2, 0, mObj->mMatWarp.size().width / 2, mObj->mMatWarp.size().height));
	
	//warpPerspective(mMat1Flip, warped, mHomography, mMat1Flip.size());
	
	mObj->mMat1Flip.copyTo(imageROI);
	
	return mObj->mMatWarp;
}

/*
 * Start some image collections now with a callback to another fucntion
 */

void CameraPair::collectCalibrationImages( boost::function<void (ThreadStatus)> f){
	
	mObj->objectPoints.clear();
	mObj->imagePoints[0].clear();
	mObj->imagePoints[1].clear();
	mObj->pWorkerThread = new boost::thread(&CameraPair::_collectCalibrationImages, this, f);
	cout << "Leeds - Collecting calibrations images for current pair" << endl;
}
	
/*
 * A threaded Function that adds lots of pairs then performs the stereo calibration then quits
 */

void CameraPair::_collectCalibrationImages(boost::function<void (ThreadStatus)> f){

	ThreadStatus t;
	t.id = sSeed;
	t.state = THREAD_START;
	f(t);
	t.state = THREAD_IMAGES;
	while(mObj->objectPoints.size() < mObj->mConfig.maxImages){
		// Add flipped images 
		t.progress =  (float)mObj->objectPoints.size() / (float)mObj->mConfig.maxImages * 100.0;
		if (mObj->mConfig.portrait){
			if(addPair(mObj->mMat0Flip, mObj->mMat1Flip,mObj->mBoard0, mObj->mBoard1)){
				sleep(1);
			}
		} else{
			if(addPair(mObj->mMat0, mObj->mMat1,mObj->mBoard0, mObj->mBoard1)){
				sleep(1);
			}
		}
		f(t);
	}
	t.state = THREAD_CALIBRATING;
	f(t);
	
	computeExtrinsics();
	t.state = THREAD_ENDED;
	f(t);
}
	
	
void CameraPair::computeExtrinsics() {
	IntrinsicParameters i0 = mObj->mUVC0->getIP();
	IntrinsicParameters i1 = mObj->mUVC1->getIP();
	
	Size s = mObj->mConfig.portrait ?  mObj->mMat0Flip.size() :  mObj->mMat0.size();
	
	double rms = stereoCalibrate(mObj->objectPoints, mObj->imagePoints[0], mObj->imagePoints[1],
					i0.M, i0.D,
                    i1.M, i1.D,
					s, mObj->mEP.R,mObj->mEP.T,mObj->mEP.E,mObj->mEP.F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_INTRINSIC +
                    CV_CALIB_RATIONAL_MODEL);
                    
        
    stereoRectify(i0.M, i0.D,
                  i1.M, i1.D,
                  s, mObj->mEP.R,mObj->mEP.T,  mObj->mEP.R1,  mObj->mEP.R2,  mObj->mEP.P1,  mObj->mEP.P2,  mObj->mEP.Q,
                  CALIB_ZERO_DISPARITY, 1, s, &mObj->validRoi[0], &mObj->validRoi[1]);
                    
    cout << "Leeds - Computed Extrinsic Parameters with " << rms << " error" << endl;
}

cv::Mat CameraPair::getMat0Trans() {
	
	Mat warped (mObj->mMat0Flip.size(), CV_8UC3);
	Mat imageROI = mObj->mMatWarp(Rect(0, 0, mObj->mMatWarp.size().width / 2, mObj->mMatWarp.size().height));
	warpPerspective(mObj->mMat0Flip, warped, mObj->mHomography, mObj->mMat0Flip.size());
	
	return warped;
}

/*
 * Returns a calibration image - the detected board matrices essentially
 */

cv::Mat CameraPair::getCalibrationImage() {
	
	if (mObj->mConfig.portrait) {
	
		Mat result (Size(mObj->mConfig.camSize.height * 2, mObj->mConfig.camSize.width), CV_8UC3);
		Mat roi = result(Rect(0,0,mObj->mConfig.camSize.height, mObj->mConfig.camSize.width));
		mObj->mBoard0.copyTo(roi);
	
		Mat roi2 = result(Rect(mObj->mConfig.camSize.height,0,mObj->mConfig.camSize.height, mObj->mConfig.camSize.width));
		mObj->mBoard1.copyTo(roi2);
	
		return result;
	}
	
	Mat result (Size(mObj->mConfig.camSize.width, mObj->mConfig.camSize.height * 2), CV_8UC3);
	Mat roi = result(Rect(0,0,mObj->mConfig.camSize.width, mObj->mConfig.camSize.height));
	mObj->mBoard0.copyTo(roi);
	
	Mat roi2 = result(Rect(0,mObj->mConfig.camSize.height,mObj->mConfig.camSize.width, mObj->mConfig.camSize.height));
	mObj->mBoard1.copyTo(roi2);
	
	return result;
}


// http://www.boost.org/doc/libs/1_49_0/doc/html/function/tutorial.html#id1529542
// http://antonym.org/2009/05/threading-with-boost---part-i-creating-threads.html



/*
 * Attempt to add a pair or none at all if we are calibrating two cameras
 */

bool CameraPair::addPair(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &board0, cv::Mat &board1) {
	vector<Point2f> corners0;
	vector<Point2f> corners1;
	
	if (findChessboard(cam0,corners0,board0, mObj->mConfig)){
		if (findChessboard(cam1,corners1,board1, mObj->mConfig)){
			mObj->objectPoints.push_back( std::vector<cv::Point3f>() );
		
			for(int j=0;j<mObj->mConfig.boardSize.height *  mObj->mConfig.boardSize.width; j++)
				mObj->objectPoints.back().push_back(Point3f(j/mObj->mConfig.boardSize.width, j%mObj->mConfig.boardSize.width, 0.0f));

			mObj->imagePoints[1].push_back(corners0);
			mObj->imagePoints[0].push_back(corners1);		
		
			return true;
		}
	}
	return false;
}

/*
 * Find the Epilines in the second image
 */
/*
cv::Mat CameraPair::findEpilines() {
	mCalibrator0.clear();
	Mat lines;
	
	Mat final = Mat(mMat1Flip);
	
	mCalibrator0.findEpilines(mMat0Flip, mMat1Flip, final, lines);
	
	cout << lines << endl;

	for (int i=0; i < lines.rows; i ++){
		float a = lines.ptr<float>(i)[0];
		float b = lines.ptr<float>(i)[1];
		float c = lines.ptr<float>(i)[2];
		
		//ax + by + c = 0 -> by = -ax - c  y = (-ax - c) / b;
		// Take x to be 0 and width
		
		cv::Point2f p0 = cv::Point2f(0.0, -c / b);
		cv::Point2f p1 = cv::Point2f(mMat1.size().width, (-mMat1.size().width * a - c) / b );
		
		cv::line(final, p0, p1, cv::Scalar(215, 29, 35));
	}
		
	return final;
}
*/



void CameraPair::computeDisparity(cv::Mat &m0, cv::Mat &m1, cv::Mat &disp, DisparityMethod m) {
	
	gpu::GpuMat gpuGrey1, gpuGrey2;
		
	Mat leftGray(mObj->mMat0.size(),CV_8UC1);
	Mat rightGray(mObj->mMat1.size(),CV_8UC1);

	cvtColor(mObj->mMat0, leftGray, CV_RGB2GRAY);
    cvtColor(mObj->mMat1, rightGray, CV_RGB2GRAY);	
	
	gpuGrey1.upload(leftGray);
	gpuGrey2.upload(rightGray);
	
    gpu::GpuMat d_disp(mObj->mMat0.size(), CV_8UC1);
    
    
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

void CameraPair::reproject(cv::Mat &disp, cv::Mat &reprojection) {
	gpu::GpuMat gpuDisp, gpuFinal;
	gpuDisp.upload(disp);
	Mat q;
	mObj->mEP.Q.convertTo(q,CV_32F,1.0);
	gpu::reprojectImageTo3D(gpuDisp, gpuFinal, q);
	gpuFinal.download(reprojection);

}

/*
 * Load a homography matrix
 */

void  CameraPair::loadHomography(string filename) {
	try {
		FileStorage fs(filename.c_str(), CV_STORAGE_READ);
		if(!fs.isOpened()) {
			cout << "Leeds - Failed to open homography file "  << filename << endl;
			return;
		}
		 
		fs["M"] >> mObj->mHomography;
		
		fs.release();
	} catch(...) {
		cout << "Leeds - failed to load homography matrix. You will need to calibrate." << endl;
	}
}


/*
 * Save the Homography
 */

 void CameraPair::saveHomography(string filename) {
	filename = filename.find("none") != string::npos ? mObj->mHomographyFile : filename;
	FileStorage fs(filename, CV_STORAGE_WRITE); 
    if( fs.isOpened() ) {
        fs << "M" << mObj->mHomography;
        fs.release();
    }
    else
        cout << "Error: can not save the homography matrix\n";
}


