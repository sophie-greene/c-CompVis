
#include "geometric_calibration.h"


#define CAM_NUM 8

//using namespace std;

//using namespace cv;

/****************************************************************
Read camera matrix and distorsion coefficients from file

Input:
	file_name: file.yml 
	cameraMatrix: cv::Mat where to store the camera matrix
	distCoeffs: cv::Mat where to store the distorsion coefficients
*****************************************************************/
void read_intrinsic_parameters(std::string file_name, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
	
	cv::FileStorage fs(file_name.c_str(), cv::FileStorage::READ);

	
	fs["cameraMatrix"] >>  cameraMatrix;
	fs["DistCoeffs"] >>  distCoeffs;

	//std::cout	<< "camera matrix: " << cameraMatrix << std::endl
	//		<< "distortion coeffs: " << distCoeffs << std::endl;

fs.release();
}

/****************************************************************
Read rotation matrix and translation std::vector from file

Input:
	file_name: file.yml 
	trans: cv::Mat where to store the translation std::vector
	rot: cv::Mat where to store the rotation matrix
*****************************************************************/
void read_extrinsic_parameters(std::string file_name,cv::Mat &trans, cv::Mat &rot) {
	
	cv::FileStorage fs(file_name.c_str(), cv::FileStorage::READ);

	
	fs["translationstd::vector"] >>  trans;
	fs["rotationMatrix"] >>  rot;

	//std::cout	<< "rotation matrix: " << rot << std::endl
	//		<< "translation std::vector: " << trans << std::endl;

fs.release();
}

/*******************************************************************
Fucntion that call the above functions to real all information
related the the camera calibration. 

Input:	
	path: path to the .yml files where to find the calibration information
Output:
	cam_paramas_t structure std::vector containing the intrinsics and extrinsics 
	and the position for each camera
****************************************************************/
std::vector<cam_params_t> read_cameras_calibration(std::string path) {

	std::string heading = "cam";
	std::string intrinsic = "Intrinsics.yml";
	std::string extrinsic = "Extrinsic.yml";
	std::string filename;

	
	
	std::vector<cam_params_t> params_list;
	for (int i = 0; i < CAM_NUM; i++) {
		cam_params_t params;
		std::stringstream ss;
		ss << i;
		std::string counter = ss.str();
		filename = heading + counter + intrinsic;
		read_intrinsic_parameters(path+filename, params.cam, params.dist);
		filename = heading + counter + extrinsic;
		read_extrinsic_parameters(path+filename, params.trans, params.rot);
		
		params.pos = -params.rot.t() * params.trans;

		params_list.push_back(params);

		//std::cout << "position: " << params.pos << std::endl;


	}
	
	return params_list;
}
