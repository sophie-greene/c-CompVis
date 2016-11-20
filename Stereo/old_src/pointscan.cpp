/**
* @brief Detection of a bright point and triangulation
* @file pointscan.cpp
* @date 26/04/2012
*
*/

#include "pointscan.hpp"

using namespace std;
using namespace boost;
using namespace cv;


void PointScan::setup (GlobalConfig &config) { 
	mObj.reset(new SharedObj(config));
}


bool PointScan::detectPoint(cv::Mat &data, cv::Mat &result, cv::Point2f &point) {
	
	Mat grey = Mat(data.size(), CV_8UC1);
	Mat thresh = Mat(data.size(), CV_8UC1);
	cvtColor( data, grey, CV_RGB2GRAY );
	result = Mat::zeros(data.rows, data.cols, CV_8UC3);
	vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    bool found = false;
	//adaptiveThreshold(grey,thresh,220,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,3,1.0);
	threshold(grey, thresh, mObj->mConfig.blockSize, 255.0, THRESH_BINARY);
	
/*	findContours( thresh, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0;
    if (hierarchy.size() > 0) {
		for( ; idx >= 0; idx = hierarchy[idx][0] ) {
			Scalar color( 255, 0, 0 );
			drawContours( result, contours, idx, color, CV_FILLED, 8, hierarchy );
			//point = 
		}
	}*/
	
	
	cvtColor( thresh, result, CV_GRAY2RGB );
	
	for (int i=0; i < thresh.rows; i++){
	
		for (int j=0; j < thresh.cols; j++){
			if (thresh.ptr<uint8_t>(i)[j] > 200){
				float score = thresh.ptr<uint8_t>(i)[j];
				// We have a local area so search it  
				for (int k =-3; k <4; k ++){
					for (int l = -3; l < 4; l++){
						score += thresh.ptr<uint8_t>(i + k)[j + l];
					}
				}
				
			//	cout << score << endl;
				if ( score / 49.0 > 50) { 
					found = true;
					point = Point2f(j,i);
					circle(result, point, 20, Scalar(255,0,0),2);	
				}			
			}
		}
	}
	

	return found;
	
}

/*
 * Given two points and an EP, compute where the depth would be
 */

Vec3f PointScan::getDepth(cv::Point2f point0, cv::Point2f point1, ExtrinsicParameters ep) {
	// Take computed X,Y as pixel co-ords to draw in for our depth map for now
	// Q * [x,y,disparity,1] = [X,Y,Z,W]
	Vec4f v;
	Vec4f p;
	
	// The below reduces this to a scalar value assuming a horizontal disparity
	float d = sqrt((point0.x - point1.x) * (point0.x - point1.x) + (point0.y - point1.y) * (point0.y - point1.y));
	v[0] = point0.x;
	v[1] = point0.y;
	v[2] = d;
	v[3] = 1;
	
	Mat r(v);
	r.mul(ep.Q);
	
	float w = r.ptr<float>(0)[3];
	float x = (r.ptr<float>(0)[0] / w);
	float y = (r.ptr<float>(0)[1] / w);
	float z = (r.ptr<float>(0)[2] / w);
	
	
	Vec3f f(x,y,z);
	
	return f;
	
}
