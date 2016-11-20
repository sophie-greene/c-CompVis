/**
* @brief Scan for a point and triangulate
* @file pointscan.hpp
* @date 26/04/2012
*
*/


#ifndef _POINTSCAN_HPP_
#define _POINTSCAN_HPP_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include "config.hpp"
#include "utils.hpp"


class PointScan {

public:
	void setup (GlobalConfig &config);

	cv::Vec3f getDepth(cv::Point2f point0, cv::Point2f point1, ExtrinsicParameters ep);
	bool detectPoint(cv::Mat &data, cv::Mat &result, cv::Point2f &point);
	
protected:

	struct SharedObj {
	
		SharedObj(GlobalConfig &config) : mConfig(config) {};
		
		GlobalConfig &mConfig;
		
	};

	boost::shared_ptr<SharedObj> mObj;

};

#endif



