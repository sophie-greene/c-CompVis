/**
* @brief Calibrator for a single camera. Removes lens distortion
* @file calibrator.hpp
* @date 26/04/2012
*
*/

#ifndef _CALIBRATOR_HPP_
#define _CALIBRATOR_HPP_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "utils.hpp"
#include "uvc_camera.hpp"

/*
 * A threaded calibrator for the single camera error values
 */

class CalibratorCamera {
public:
	CalibratorCamera(CameraParameters &p, cv::Size &isize, cv::Size &bsize) : mP(p), mImageSize(isize), mBoardSize(bsize) {};
	
	virtual double operator ()(uint8_t threadCount); // Called when performing calibration
	
	bool addImage(cv::Mat &cam, cv::Mat &board);
	
	void clear() { imagePoints.clear(); objectPoints.clear(); };
	
protected:

	CameraParameters &mP;
	cv::Size &mImageSize;
	cv::Size &mBoardSize;

	std::vector< std::vector<cv::Point2f> > imagePoints;
	std::vector< std::vector<cv::Point3f> > objectPoints;

};


/*
 * A threaded calibrator for the single camera error values
 */

class CalibratorWorld : public CalibratorCamera {
public:
	CalibratorWorld(CameraParameters &ip, cv::Size &isize, cv::Size &bsize) : CalibratorCamera(ip,isize,bsize) {};
	
	double operator ()(uint8_t threadCount); // Called when performing calibration
	
};


/*
 * 
 */

#endif
