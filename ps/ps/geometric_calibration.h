#ifndef GEOMETRIC_CALIBRATION_H
#define GEOMETRIC_CALIBRATION_H
#include <string>
#include <vector>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>

#define CAM_NUM 8

//structure containing information related with the camera parameters: 
//		extrinsic parameters (rotation and translation),
//		intrinsic parameters (distorsion coeffs and camera matrix)
//		position of the camera relative to world origin
struct cam_params_t{
	cv::Mat rot;
	cv::Mat trans;
	cv::Mat cam;
	cv::Mat dist;
	cv::Mat pos;
};

void read_intrinsic_parameters(std::string file_name,cv::Mat &cam, cv::Mat &dist);
void read_extrinsic_parameters(std::string file_name,cv::Mat &rot, cv::Mat &trans);
std::vector<cam_params_t> read_cameras_calibration(std::string path);

#endif /* GEOMETRIC_CALIBRATION_H */
