#include "utils.h"

/* This file includes general functions that can be used in several parts of the code */

//using namespace cv;
using namespace std;

/*******************************************************************
Compute rotation/translation compound matrix 
r11 r12 r13 t1
r21 r22 213 t2
r31 r32 r33 t3
0   0   0   1
Input: 
	rot: rotation matrix
	trans: translation vector
Output: 
	rotation+translation compound matrix
*********************************************************************/

cv::Mat roto_translation(cv::Mat rot, cv::Mat trans) {

	cv::Mat M = cv::Mat::zeros(4,4,CV_64F);
	
	double *output = (double*)(M.data);
	double *input_rot = (double*)(rot.data);
	double *input_trans = (double*)(trans.data);
	//rotation
	for(int j = 0;j < 3;j++){
		for(int i = 0;i < 3;i++){
			output[M.cols * j + i ] = input_rot[rot.cols*j+i];
		}
	}
	//translation
	output[M.cols * 0 + 3 ] = input_trans[trans.cols*0];
	output[M.cols * 1 + 3 ] = input_trans[trans.cols*1];
	output[M.cols * 2 + 3 ] = input_trans[trans.cols*2];
	
	//homogeneus
	output[M.cols * 3 + 3 ] = 1.0;

	
	return M;
	
}

/********************************************************************
Calculates norm of m 
Input:
	m: vector
	size: number of elements of m taken into account to compute the norm
Output:
	norm of m
**********************************************************************/
double calc_norm(cv::Mat m, int size) {
	
	double * data = (double*)(m.data);
	double n = 0;
	for (int i = 0; i < size; i++) {
		n += data[i]*data[i];
	}
	return sqrt(n);

}

/**********************************************************************************
Computes essential matrix of a camera pair given extrinsic parameters of each camera 
(rotation and translation)

Input:	
	r1 and t1: rotation and translation of camera 1
	r2 and t2: rotation and translation of camera 2
Output:
	Essential matrix
**********************************************************************************/
cv::Mat calc_essential_matrix(cv::Mat r1, cv::Mat t1, cv::Mat r2, cv::Mat t2 ) {

	cv::Mat Mw1,Mw2,M,R,T,S,E,temp;
	//rototranslation matrices for each camera
	Mw1 = roto_translation(r1,t1);
	Mw2 = roto_translation(r2,t2);
	M = Mw1*Mw2.inv();
	
	temp = M(cv::Rect(0,0,3,3));
	temp.copyTo(R);
	temp = M(cv::Rect(3,0,1,3));
	temp.copyTo(T);
	S = cv::Mat::zeros(3,3,CV_64F);
	double* s_data = (double*) (S.data);
	double* t_data = (double*) (T.data);
	
	
	s_data[1] = -t_data[2];
	s_data[2] = t_data[1];
	s_data[3] = t_data[2];
	s_data[5] = -t_data[0];
	s_data[6] = -t_data[1];
	s_data[7] = t_data[0];
	
	E = R.t()*S;
	
	//cout<< E << endl;
		
	return E;
	
}

/******************************************************************************
Computes camera transformation matrix: cam*P where
cam is the camera matrix and P is r11 r12 r13 t1
								r21 r22 213 t2
								r31 r32 r33 t3
Input:
	cam: camera matrix 
	rot: rotation matrix
	trans: translation vector
Output:
	camera transformation matrix
*********************************************************************************/

cv::Mat compound_cam_transformation(cv::Mat cam, cv::Mat rot, cv::Mat trans) {
	
	cv::Mat P(3,4,CV_64F);
	
	P.at<double>(0) = rot.at<double>(0);
	P.at<double>(1) = rot.at<double>(1);
	P.at<double>(2) = rot.at<double>(2);
	P.at<double>(4) = rot.at<double>(3);
	P.at<double>(5) = rot.at<double>(4);
	P.at<double>(6) = rot.at<double>(5);
	P.at<double>(8) = rot.at<double>(6);
	P.at<double>(9) = rot.at<double>(7);
	P.at<double>(10) = rot.at<double>(8);

	P.at<double>(3) = trans.at<double>(0);
	P.at<double>(7) = trans.at<double>(1);
	P.at<double>(11) = trans.at<double>(2);
	
	return cam*P;

}
/*
cv::Mat calc_intersection(cv::Mat p0, cv::Mat d0, cv::Mat p1, cv::Mat d1) {
	double a = d0.dot(d0);
    double b = d0.dot(d1);
    double c = d1.dot(d1);
	double d = d0.dot(p0-p1);
	double e = d1.dot(p0-p1);
	double f = (p0-p1).dot(p0-p1);

	double sc = (b*e-c*d)/(a*c-b*b);
    double tc = (a*e-b*d)/(a*c-b*b);

    p0 = p0+sc*d0;
    p1 = p1+tc*d1;
    cv::Mat intersection = (p0 + p1)/ 2;

	return intersection;
}
*/
