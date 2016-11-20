/**
* @brief Stereo Calibration Body
* @file stereo_calibration.cpp
* @date 26/04/2012
*
*/
   
#include "stereo_calibration.hpp"

using namespace cv;
using namespace std;
using namespace boost;


StereoCalibration::StereoCalibration() {
		
}

void StereoCalibration::setup(GlobalConfig &config){
	mObj.reset(new SharedObj(config)); 
	
	mObj->imagePoints = new vector< vector<Point2f> > [2];
	mObj->validRoi = new Rect[2];
	mObj->mSquareSize = 1.0f;	
}

StereoCalibration::~StereoCalibration(){
//	delete imagePoints;
}

/* 
 * Given an image find the chessboard - board contains the result and can be 
 */

bool StereoCalibration::findChessboard(Mat &cam0, vector<Point2f> &corners, Mat &board ) {

	if ( findChessboardCorners(cam0, mObj->mConfig.boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK) ) {
	
		// Update the view
		
		cam0.copyTo(board);
		drawChessboardCorners(board, mObj->mConfig.boardSize, corners, true);
		
		Mat grey = cam0;

		cvtColor( cam0, grey, CV_RGB2GRAY);		
		cornerSubPix(grey, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));

		return true;
	}

	cam0.copyTo(board);
	
	return false;
}

void StereoCalibration::clear() {
	mObj->imagePoints[0].clear();
	mObj->imagePoints[1].clear();
	mObj->objectPoints.clear();
}

///\todo cx that this actually makes a proper copy of the points when added

/*
 * Add a set of calibration points to our calibration routine
 */

void StereoCalibration::addToCalibration(bool right, vector<Point2f> &corners) {

	for(int j=0;j<mObj->mConfig.boardSize.height *  mObj->mConfig.boardSize.width; j++)
       mObj->objectPoints.back().push_back(Point3f(j/mObj->mConfig.boardSize.width, j%mObj->mConfig.boardSize.width, 0.0f));

	if (right)
		mObj->imagePoints[1].push_back(corners);
	else
		mObj->imagePoints[0].push_back(corners);

}

/*
 * Add a single image if we are calibrating one camera on its own
 */

bool StereoCalibration::addSingle(cv::Mat &cam, cv::Mat &board) {
	vector<Point2f> corners;
	if (findChessboard(cam,corners,board)){
		mObj->objectPoints.push_back( std::vector<cv::Point3f>() );
		addToCalibration(false,corners);
		return true;
	}
	return false;
}

/*
 * Attempt to add a pair or none at all if we are calibrating two cameras
 */

bool StereoCalibration::addPair(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &board0, cv::Mat &board1) {
	vector<Point2f> corners0;
	vector<Point2f> corners1;
	
	if (findChessboard(cam0,corners0,board0)){
		if (findChessboard(cam1,corners1,board1)){
			mObj->objectPoints.push_back( std::vector<cv::Point3f>() );
			
			addToCalibration(false,corners0);
			addToCalibration(true,corners1);

			mObj->imagePoints[1].push_back(corners0);
			mObj->imagePoints[0].push_back(corners1);		
		
			return true;
		}
	}
	return false;
}


float StereoCalibration::calibrateSingle(CameraParameters &in) {
	
	in.M.ptr<float>(0)[0] = (float)mObj->mConfig.camSize.width / (float)mObj->mConfig.camSize.height;
	in.M.ptr<float>(1)[1] = (float)mObj->mConfig.camSize.height / (float)mObj->mConfig.camSize.width;
	
    
  /*  mIP0.M.ptr<float>(0)[2] = (float)mConfig.camSize.height / 2.0;
    mIP0.M.ptr<float>(1)[2] = (float)mConfig.camSize.width / 2.0;
    
    mIP1.M.ptr<float>(0)[2] = (float)mConfig.camSize.height / 2.0;
    mIP1.M.ptr<float>(1)[2] = (float)mConfig.camSize.width / 2.0;*/

	return calibrateCamera(mObj->objectPoints,mObj->imagePoints[0], mObj->mConfig.camSize, in.M, in.D, in.Rs, in.Ts,
		CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_RATIONAL_MODEL  );
}


	
/*
 * Find the transform between two rectified images - uses a chessboard to calibrate
 */

bool StereoCalibration::findTransform(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &trans){
	vector<Point2f> corners0;
	vector<Point2f> corners1;
	
	Mat board(cam0);
	
	if (findChessboard(cam0,corners0,board)){
		if (findChessboard(cam1,corners1,board)){
			trans = findHomography(corners0,corners1);
			return true;
		}
	}
	return false;
}
/*
 * Find the fundamental Matrix and then attempt to find the epipolar lines
 * as above, takes the chessboard images and finds points
 */
 
 bool StereoCalibration::findEpilines(cv::Mat &cam0, cv::Mat &cam1, cv::Mat &points, cv::Mat &lines){
	vector<Point2f> corners0;
	vector<Point2f> corners1;
	
	Mat board (mObj->mConfig.camSize, CV_8UC3);
	
	if (findChessboard(cam0,corners0,board)){
		if (findChessboard(cam1,corners1,board)){
			
			Mat F =  findFundamentalMat(corners0, corners1, CV_FM_8POINT);
			computeCorrespondEpilines(corners0, 0, F, lines);
			
			// Draw the epilines on points
			
			points = cam1.clone();
		
			for (int i=0; i < lines.rows; i ++){
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
 * CALIBRATION QUALITY CHECK
 * because the output fundamental matrix implicitly
 * includes all the output information,
 * we can check the quality of calibration using the
 * epipolar geometry constraint: m2^t*F*m1=0
 */

void StereoCalibration::runStandardCalibrationCheck(CameraParameters &in0, CameraParameters &in1, ExtrinsicParameters &ex){
    double err = 0;
    int npoints = 0;
    vector<cv::Vec3f> lines[2];
    int N = mObj->mConfig.boardSize.width * mObj->mConfig.boardSize.height * mObj->objectPoints.size();
    lines[0].resize(N);
    lines[1].resize(N);
    for( int i = 0; i < mObj->objectPoints.size(); i++ )  {
        int npt = (int)mObj->imagePoints[0][i].size();
        Mat imgpt[2];
        
		imgpt[0] = Mat(mObj->imagePoints[0][i]);
		undistortPoints(imgpt[0], imgpt[0], in0.M, in0.D, Mat(), in0.M);
		computeCorrespondEpilines(imgpt[0], 1, ex.F, lines[0]);
		
		imgpt[1] = Mat(mObj->imagePoints[1][i]);
		undistortPoints(imgpt[1], imgpt[1], in1.M, in1.D, Mat(), in1.M);
		computeCorrespondEpilines(imgpt[1], 2, ex.F, lines[0]);
       
        for( int j = 0; j < npt; j++ )
        {
            double errij = fabs(mObj->imagePoints[0][i][j].x*lines[1][j][0] +
                                mObj->imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(mObj->imagePoints[1][i][j].x*lines[0][j][0] +
                                mObj->imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    
    cout << "Leeds - average reprojection err: " <<  err/npoints << endl;
}
