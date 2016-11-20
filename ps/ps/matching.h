#ifndef MATCHING_H
#define MATCHING_H


#include <string>
#include<vector>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>

#include "geometric_calibration.h"
#include "utils.h"



std::vector<std::vector<double> > calc_matching_cost(std::vector<std::vector<cv::Point> > lines,std::vector<std::vector<cv::Mat> > n1,std::vector<std::vector<cv::Mat> > n2,cam_params_t cam1,cam_params_t cam2);
std::vector<std::vector<double> > calc_matching_cost(std::vector<std::vector<cv::Point> > lines,IplImage* im1, IplImage* im2);
IplImage* visualize_normals_world (std::vector<std::vector<cv::Mat> > n1,cam_params_t params,std::string win_name1, std::string win_name2);
void visualize_matching_cost(std::vector<std::vector<double> > cost);
double Crm (int l,int r);
double Clm (int l,int r);
double Clo (int l,int r);
double Cro (int l,int r);
std::vector<std::vector<cv::Point> > find_best_path(std::vector<std::vector<double> > matching_costs,std::vector<std::vector<cv::Point> > lines,IplImage*im1, IplImage* im2, int w, int h);

#endif /* MATCHING_H */
