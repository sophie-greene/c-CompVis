


#include "light_calibration.h"
#include "geometric_calibration.h"
#include "utils.h"


using namespace std;
//using namespace cv;
using namespace pcl;

/*******************************************************************************
Calculates light directions. Lights directions are approximated as the vector going 
from P to each cameras position, being P the intersection of the 
Z axis of the current cam with the floor plane.

Input:

	params:	cameras parameters
	cam_num: index of the actual camera
	available_lights: index of the available lights for this camera
	cameras_height: vector with the height of each camera as seen along the Z axis of the camera itself
	im_w: image width
	im_h: image height

Output:
	cv::Mat containing the lights directions
*********************************************************************************/


cv::Mat calc_light_direction(vector<cam_params_t> params,int cam_num,vector<int> available_lights,vector<double> &cameras_height, int im_w, int im_h) {

	vector<vector<cv::Mat>> light_dirs;
	
	cv::Mat l(available_lights.size(),3,CV_64F);
	double *l_data = (double*)(l.data);
	float scalef = (float)im_w/640.0;
	cvNamedWindow("Test Lights");
	
	//point in the floor plane
	cv::Mat p_plane(3,1,CV_64F);
	p_plane.at<double>(0) = 0;
	p_plane.at<double>(1) = 0;
	p_plane.at<double>(2) = 0;
	
	//a point in the line can be the camera position
	cv::Mat p_line(3,1,CV_64F);
	p_line.at<double>(0) = params[cam_num].pos.at<double>(0);
	p_line.at<double>(1) = params[cam_num].pos.at<double>(1);
	p_line.at<double>(2) = params[cam_num].pos.at<double>(2);
	
	//normal vector to the floor plane
	cv::Mat n(3,1,CV_64F);
	n.at<double>(0) = 0;
	n.at<double>(1) = 0;
	n.at<double>(2) = 1;
	
	//vector going along the Z axis of the camera
	cv::Mat aux(4,1,CV_64F);
	aux.at<double>(0) = 0;
	aux.at<double>(1) = 0;
	aux.at<double>(2) = 1;
	aux.at<double>(3) = 1;
	
	cv::Mat M = roto_translation(params[cam_num].rot, params[cam_num].trans);
	//cv::Mat Pl = compound_cam_transformation(params[cam_num].cam,params[cam_num].rot,params[cam_num].trans);
	
	//change vector from camera to world coordinates
	aux = M.inv()*aux;
	
	cv::Mat l_dir(3,1,CV_64F);
	l_dir.at<double>(0) = aux.at<double>(0);
	l_dir.at<double>(1) = aux.at<double>(1);
	l_dir.at<double>(2) = aux.at<double>(2);
	
	/*
	cv::Mat p1(3,1,CV_64F);
	p1.at<double>(0) = im_w/(2*scalef);
	p1.at<double>(1) = im_h/(2*scalef);
	p1.at<double>(2) = 1;
	
	cv::Mat aux = Pl.t()*(Pl*Pl.t()).inv()*p1;
	aux /= aux.at<double>(3);
	*/
	/*
	cv::Mat l_dir(3,1,CV_64F);
		
	l_dir.at<double>(0) = aux.at<double>(0) - params[cam_num].pos.at<double>(0);
	l_dir.at<double>(1) = aux.at<double>(1) - params[cam_num].pos.at<double>(1);
	l_dir.at<double>(2) = aux.at<double>(2) - params[cam_num].pos.at<double>(2);
	
	l_dir = l_dir / calc_norm(l_dir,3);
	*/
	l_dir -= params[cam_num].pos;
	
	
	
	l_dir = l_dir / calc_norm(l_dir,3);				//this is a normal vector in the direction of the Z axis of the camera but seen in world coordinates

	double delta = (p_plane-p_line).dot(n)/l_dir.dot(n);	//compute intersection
	cv::Mat depth_p = l_dir*delta+p_line;
	
	aux.at<double>(0) = depth_p.at<double>(0);
	aux.at<double>(1) = depth_p.at<double>(1);
	aux.at<double>(2) = depth_p.at<double>(2);
	aux.at<double>(3) = 1.0;
	
	//transfor intersection point to camera coordinates
	aux = M*aux;					
	cv::Mat origin(3,1,CV_64F);
	origin.at<double>(0) = aux.at<double>(0);
	origin.at<double>(1) = aux.at<double>(1);
	origin.at<double>(2) = aux.at<double>(2);

	//use the Z coordinate of the intersection point as seen from the camera as the camera height
	cameras_height.push_back(origin.at<double>(2));
	
	//for each available light store its direction 
	for (int i = 0; i < available_lights.size(); i++) {
		
		int light_idx = available_lights[i];
	
		cv::Mat pos = params[cam_num].rot*params[light_idx].pos + params[cam_num].trans;
		
		cv::Mat dir = pos-origin;
		dir = dir / calc_norm(dir,3);
		l_data[i*3] = dir.at<double>(0);
		l_data[i*3+1] = dir.at<double>(1);
		l_data[i*3+2] = dir.at<double>(2);		
	}

	return l;
}

