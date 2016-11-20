/**
* @brief Calibrator for a single camera. Removes lens distortion
* @file calibrator.cpp
* @date 26/04/2012
*
*/

#include "calibrator.hpp"

using namespace std;
using namespace boost;
using namespace cv;

/*
 * Add a single image if we are calibrating one camera on its own
 */

bool CalibratorCamera::addImage(cv::Mat &cam, cv::Mat &board) {
	vector<Point2f> corners;
	if (findChessboard(cam,corners,board, mBoardSize)){
		objectPoints.push_back( std::vector<cv::Point3f>() );
		
		for(int j=0;j< mBoardSize.height *  mBoardSize.width; j++)
			objectPoints.back().push_back(Point3f(j/mBoardSize.width, j%mBoardSize.width, 0.0f));
	
		imagePoints.push_back(corners);
		
		return true;
	}
	return false;
}

/*
 * Callable Function - launched from a thread and performs actual calibration
 */

double CalibratorCamera::operator()(uint8_t threadCount) {
	threadCount++;
	mP.M.ptr<float>(0)[0] = (float)mImageSize.width / (float)mImageSize.height;
	mP.M.ptr<float>(1)[1] = (float)mImageSize.height / (float)mImageSize.width;
	
	double error = calibrateCamera(objectPoints,imagePoints, mImageSize, mP.M, mP.D, mP.Rs, mP.Ts,
		CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_RATIONAL_MODEL  );
		
	cerr << "Leeds - calibrated camera with error " << error << endl;

	mP.mCalibrated = true;
	threadCount--;
	return error;
}

/*
 * Callable Function - launched from a thread performing world calibration on all cameras
 */
 
 double CalibratorWorld::operator()(uint8_t threadCount) {
	threadCount++;
	vector<Point3f> tOPoints;
		
	for(int j=0;j< mBoardSize.height * mBoardSize.width; j++)
		tOPoints.push_back(Point3f(j/mBoardSize.width, j%mBoardSize.width, 0.0f));
	
	solvePnP(tOPoints,imagePoints[0],mP.M, mP.D, mP.R, mP.T, false, CV_ITERATIVE);
	
	cout << mP.R << endl;
	
	///\todo - generate reprojection error
	
	cerr << "Leeds - calibrated world" << endl;
	threadCount--;
}

/*

void Calibrator::generateExtrinsics(std::vector< boost::shared_ptr<UVCVideo> > &cams) {
	cout << "Leeds - Calibrating to world" << endl;
	mObj->imagePoints.clear();
	mObj->objectPoints.clear();
	mObj->pWorkerThread = new boost::thread(&Calibrator::_generateExtrinsics, this, cams);
	
}


void Calibrator::_generateExtrinsics(std::vector< boost::shared_ptr<UVCVideo> > &cams) {
		
	Mat board(mObj->mConfig.camSize,CV_8UC3);
	
	// Loop through all cameras finding the chessboard. If one fails start again
	vector<Point2f> corners;
	bool found;
	
	int idx;
	do{	
		found = true;
		idx = 0;

		mObj->imagePoints.clear();
		for(vector<shared_ptr<UVCVideo> >::iterator it = cams.begin(); it != cams.end(); it++){
		// Take a snap shot until we find the chessboard
			Mat snap(mObj->mConfig.camSize, CV_8UC3,(*it)->getBuffer());

			if (!findChessboard(snap,corners,board,mObj->mConfig)){
				found = false;
				cout << "Not found in camera "  << idx << endl;
			} else {
				mObj->imagePoints.push_back(corners);
		

			}
			idx ++;
		}

	} while(!found);
	
	
	// Now we have all the corners, 1 for each camera  - calibrate each set of transforms
	idx = 0;
	for(vector<shared_ptr<UVCVideo> >::iterator it = cams.begin(); it != cams.end(); it++){
	
		CameraParameters in = (*it)->getIP();

	 	vector<Point3f> tOPoints;
		
		for(int j=0;j<mObj->mConfig.boardSize.height *  mObj->mConfig.boardSize.width; j++)
			tOPoints.push_back(Point3f(j/mObj->mConfig.boardSize.width, j%mObj->mConfig.boardSize.width, 0.0f));
	
		solvePnP(tOPoints,mObj->imagePoints[idx],in.M, in.D, in.R, in.T, false, CV_ITERATIVE);
		
		cout << "Computed world transform for camera " << idx << endl;
	
		// Should only be one of each - this is the master transform for our world space
			
		(*it)->setIP(in);
		
		idx++;
	}
	
}




float Calibrator::calibrate(CameraParameters &in) {
	
	in.M.ptr<float>(0)[0] = (float)mObj->mConfig.camSize.width / (float)mObj->mConfig.camSize.height;
	in.M.ptr<float>(1)[1] = (float)mObj->mConfig.camSize.height / (float)mObj->mConfig.camSize.width;
	
    


	return calibrateCamera(mObj->objectPoints,mObj->imagePoints, mObj->mConfig.camSize, in.M, in.D, in.Rs, in.Ts,
		CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_RATIONAL_MODEL  );
}



 
 

void Calibrator::reproject(cv::Mat &result, CameraParameters &in) {
		
	vector<Point3f> objectPoints;
		
	for(int j=0;j<mObj->mConfig.boardSize.height *  mObj->mConfig.boardSize.width; j++){
		objectPoints.push_back( Point3f(j/mObj->mConfig.boardSize.width, j%mObj->mConfig.boardSize.width, 0.0f ));
	}
	
	vector<Point2f> results;
		
	projectPoints(objectPoints, in.R, in.T, in.M, in.D, results );
		
		
	/*	Rodrigues(in.R,r);
				
		Mat trans(Size(4,3),CV_64F);
		trans.zeros(Size(4,3),CV_64F);
		trans.at<double_t>(0,0) = r.at<double_t>(0,0);
		trans.at<double_t>(0,1) = r.at<double_t>(0,1);
		trans.at<double_t>(0,2) = r.at<double_t>(0,2);
		trans.at<double_t>(0,3) = in.T.at<double_t>(0,0);

		trans.at<double_t>(1,0) = r.at<double_t>(1,0);
		trans.at<double_t>(1,1) = r.at<double_t>(1,1);
		trans.at<double_t>(1,2) = r.at<double_t>(1,2);
		trans.at<double_t>(1,3) = in.T.at<double_t>(1,0);
		
		trans.at<double_t>(2,0) = r.at<double_t>(2,0);
		trans.at<double_t>(2,1) = r.at<double_t>(2,1);
		trans.at<double_t>(2,2) = r.at<double_t>(2,2);
		trans.at<double_t>(2,3) = in.T.at<double_t>(2,0);
	
		p = pointMul(trans,p);
		
		// Now draw some circles on the screen where these points are
	
	
	for (vector<Point2f>::iterator it = results.begin(); it != results.end(); it++)
		circle(result, (*it), 5, Scalar(255,0,0),2);	
		
}

 
cv::Point3f Calibrator::solveForAll(std::vector< boost::shared_ptr<UVCVideo> > &cams, std::vector<cv::Point2f> points) {

	Mat m (Size(3 + cams.size(), 3 * cams.size()), CV_64FC1);
	Mat s (Size(1,4), CV_64FC1);
	Mat c (Size(1,3 * cams.size()), CV_64FC1);

	int idx = 0;
	int idy = 0;
				
	for (std::vector< boost::shared_ptr<UVCVideo> >::iterator it = cams.begin(); it != cams.end(); it++){
		CameraParameters in = (*it)->getIP();
	
		Mat r (Size(3,3), CV_64FC1);
		Rodrigues(in.R,r);
		
		vector<Point2f> results;
		vector<Point2f> tpoints;
		tpoints.push_back(points[idy]);
		undistortPoints(tpoints,results, in.M, in.D);
	
		// now create the solving matrices
		
		m.at<double_t>(idx,0) = r.at<double_t>(0,0);
		m.at<double_t>(idx,1) = r.at<double_t>(0,1);
		m.at<double_t>(idx,2) = r.at<double_t>(0,2);
		
		m.at<double_t>(idx + 1,0) = r.at<double_t>(1,0);
		m.at<double_t>(idx + 1,1) = r.at<double_t>(1,1);
		m.at<double_t>(idx + 1,2) = r.at<double_t>(1,2);
		
		m.at<double_t>(idx + 2,0) = r.at<double_t>(2,0);
		m.at<double_t>(idx + 2,1) = r.at<double_t>(2,1);
		m.at<double_t>(idx + 2,2) = r.at<double_t>(2,2);
		
		// special values depending on idx
		
		for (int i = 0; i < cams.size(); i ++) {
			
			if (i == idy) {
				m.at<double_t>(idx,3 + i) = -results[0].x;
				m.at<double_t>(idx+1,3 + i) = -results[0].y;
				m.at<double_t>(idx+2,3 + i) = -1.0;
			}
			else {
				m.at<double_t>(idx,3+ i) = 0;
				m.at<double_t>(idx+1,3+ i) = 0;
				m.at<double_t>(idx+2,3+ i) = 0;
			}
		}
		
		c.at<double_t>(idx,0) = -in.T.at<double_t>(0,0);
		c.at<double_t>(idx+1,0) = -in.T.at<double_t>(1,0);
		c.at<double_t>(idx+2,0) = -in.T.at<double_t>(2,0);
	
		idx += 3;
		idy++;
	}
	// Should only be one. Now solve and send back 

	solve(m,c,s,DECOMP_QR);
	
	Point3f p(s.at<double_t>(0,0),s.at<double_t>(1,0),s.at<double_t>(2,0));
	
	cout << p << endl;
	
	return p;
}

void Calibrator::testSystem(std::vector< boost::shared_ptr<UVCVideo> > &cams, Point3f test) {
	vector<Point3f> tOPoints;
	tOPoints.push_back(test);
	
	Mat m (Size(3 + cams.size(), 3 * cams.size()), CV_64FC1);
	Mat s (Size(1,4), CV_64FC1);
	Mat c (Size(1,3 * cams.size()), CV_64FC1);

	
	int idx = 0;
	int idy = 0;

	for (std::vector< boost::shared_ptr<UVCVideo> >::iterator it = cams.begin(); it != cams.end(); it++){
		CameraParameters in = (*it)->getIP();
		vector<Point2f> results;
		projectPoints(tOPoints, in.R, in.T, in.M, in.D, results );
			
		Mat r (Size(3,3), CV_64FC1);
		Rodrigues(in.R,r);
		
		vector<Point2f> upoints;
		undistortPoints(results,upoints, in.M, in.D);
					
		// now create the solving matrices
		
		m.at<double_t>(idx,0) = r.at<double_t>(0,0);
		m.at<double_t>(idx,1) = r.at<double_t>(0,1);
		m.at<double_t>(idx,2) = r.at<double_t>(0,2);
		
		m.at<double_t>(idx + 1,0) = r.at<double_t>(1,0);
		m.at<double_t>(idx + 1,1) = r.at<double_t>(1,1);
		m.at<double_t>(idx + 1,2) = r.at<double_t>(1,2);
		
		m.at<double_t>(idx + 2,0) = r.at<double_t>(2,0);
		m.at<double_t>(idx + 2,1) = r.at<double_t>(2,1);
		m.at<double_t>(idx + 2,2) = r.at<double_t>(2,2);
		
		// special values depending on idx
		
		for (int i = 0; i < cams.size(); i ++) {
			
			if (i == idy) {
				m.at<double_t>(idx,3 + i) = -upoints[0].x;
				m.at<double_t>(idx+1,3 + i) = -upoints[0].y;
				m.at<double_t>(idx+2,3 + i) = -1.0;
			}
			else {
				m.at<double_t>(idx,3+ i) = 0;
				m.at<double_t>(idx+1,3+ i) = 0;
				m.at<double_t>(idx+2,3+ i) = 0;
			}
		}
		
		c.at<double_t>(idx,0) = -in.T.at<double_t>(0,0);
		c.at<double_t>(idx+1,0) = -in.T.at<double_t>(1,0);
		c.at<double_t>(idx+2,0) = -in.T.at<double_t>(2,0);
	
		idx += 3;
		idy++;
	}
	// Should only be one. Now solve and send back 
	
	cout << "*********************" << endl << m << endl << c << endl;
	
	solve(m,c,s,DECOMP_QR);
	
	cout << "Final" <<  s << endl;
	
	
	// Plug values back into the cameras and test
	
	for (std::vector< boost::shared_ptr<UVCVideo> >::iterator it = cams.begin(); it != cams.end(); it++){
		
		CameraParameters in = (*it)->getIP();
		
		vector<Point2f> results;
		projectPoints(tOPoints, in.R, in.T, in.M, in.D, results );
		vector<Point2f> upoints;
		undistortPoints(results,upoints, in.M, in.D);
			
		Mat r (Size(3,3), CV_64FC1);
		Rodrigues(in.R,r);
		Mat f (Size(4,3), CV_64FC1);	


		Mat m (Size(3,3), CV_64FC1);

		Mat c (Size(1,4), CV_64FC1);
		
		cout << in.T << endl;
		
		// Plug in our point 
		
		c.at<double_t>(0,0) = test.x;
		c.at<double_t>(1,0) = test.y;
		c.at<double_t>(2,0) = test.z;
		c.at<double_t>(3,0) = 1.0;

		// Create the 4x3 matrix we are using in the main
		
		f.at<double_t>(0,0) = r.at<double_t>(0,0);
		f.at<double_t>(0,1) = r.at<double_t>(0,1);
		f.at<double_t>(0,2) = r.at<double_t>(0,2);
		f.at<double_t>(0,3) = in.T.at<double_t>(0,0);
		
		f.at<double_t>(1,0) = r.at<double_t>(1,0);
		f.at<double_t>(1,1) = r.at<double_t>(1,1);
		f.at<double_t>(1,2) = r.at<double_t>(1,3);
		f.at<double_t>(1,3) = in.T.at<double_t>(1,0);
		
		f.at<double_t>(2,0) = r.at<double_t>(2,0);
		f.at<double_t>(2,1) = r.at<double_t>(2,1);
		f.at<double_t>(2,2) = r.at<double_t>(2,2);
		f.at<double_t>(2,3) = in.T.at<double_t>(2,0);

	//	f = in.M *f;
	
		c = f * c;
		
		c = c * (1.0/ c.at<double_t>(2,0));
		
	//	c.at<double_t>(0,0) = c.at<double_t>(0,0) * in.M.at<double_t>(0,0) + in.M.at<double_t>(0,2);
	//	c.at<double_t>(1,0) = c.at<double_t>(1,0) * in.M.at<double_t>(1,1) + in.M.at<double_t>(1,2);
		
		
		
		cout << "Test Matrix" << c << endl;
		
		cout << "Should match" << endl;
		cout << upoints[0].x << "," << upoints[0].y << endl;
		
	
	}
	
}	

void Calibrator::testCamera( boost::shared_ptr<UVCVideo> &cam, cv::Point3f test, cv::Mat &result){
	
	vector<Point3f> tOPoints;
	tOPoints.push_back(test);
	
	int idx = 0;
	CameraParameters in = cam->getIP();
	vector<Point2f> results;
	projectPoints(tOPoints, in.R, in.T, in.M, in.D, results );
	
	circle(result, results[0], 8, Scalar(0,255,0),2);	
	vector<Point2f> upoints;
	undistortPoints(results,upoints, in.M, in.D);
	
	cout << "Test gives " << results[0].x <<","<< results[0].y << " maps to "  << upoints[0].x << "," << upoints[0].y << endl;
	
}
*/
