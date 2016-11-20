
#include "geometric_calibration.h"


#define CAM_NUM 8

using namespace std;

//using namespace cv;

/****************************************************************
Read camera matrix and distorsion coefficients from file

Input:
	file_name: file.yml 
	cameraMatrix: cv::Mat where to store the camera matrix
	distCoeffs: cv::Mat where to store the distorsion coefficients
*****************************************************************/
void read_intrinsic_parameters(string file_name, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
	
	cv::FileStorage fs(file_name.c_str(), cv::FileStorage::READ);

	
	fs["cameraMatrix"] >> cameraMatrix;
	fs["DistCoeffs"] >> distCoeffs;

	//cout	<< "camera matrix: " << cameraMatrix << endl
	//		<< "distortion coeffs: " << distCoeffs << endl;

fs.release();
}

/****************************************************************
Read rotation matrix and translation vector from file

Input:
	file_name: file.yml 
	trans: cv::Mat where to store the translation vector
	rot: cv::Mat where to store the rotation matrix
*****************************************************************/
void read_extrinsic_parameters(string file_name,cv::Mat &trans, cv::Mat &rot) {
	
	cv::FileStorage fs(file_name.c_str(), cv::FileStorage::READ);

	
	fs["translationVector"] >> trans;
	fs["rotationMatrix"] >> rot;

	//cout	<< "rotation matrix: " << rot << endl
	//		<< "translation vector: " << trans << endl;

fs.release();
}

/*******************************************************************
Fucntion that call the above functions to real all information
related the the camera calibration. 

Input:	
	path: path to the .yml files where to find the calibration information
Output:
	cam_paramas_t structure vector containing the intrinsics and extrinsics 
	and the position for each camera
****************************************************************/
vector<cam_params_t> read_cameras_calibration(string path) {

	string heading = "cam";
	string intrinsic = "Intrinsics.yml";
	string extrinsic = "Extrinsic.yml";
	string filename;

	
	
	vector<cam_params_t> params_list;
	for (int i = 0; i < CAM_NUM; i++) {
		cam_params_t params;
		stringstream ss;
		ss << i;
		string counter = ss.str();
		filename = heading + counter + intrinsic;
		read_intrinsic_parameters(path+filename, params.cam, params.dist);
		filename = heading + counter + extrinsic;
		read_extrinsic_parameters(path+filename, params.trans, params.rot);
		
		params.pos = -params.rot.t() * params.trans;

		params_list.push_back(params);

		//cout << "position: " << params.pos << endl;


	}
	
	return params_list;
}
