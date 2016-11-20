#ifndef EPIPOLAR_CORRESPONDENCE_H
#define EPIPOLAR_CORRESPONDENCE_H


#include <string>
#include<vector>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>


std::vector<std::vector<cv::Point>> find_epipolar_lines(cv::Mat E, int u, int v,  cv::Mat cam1, cv::Mat cam2, IplImage *im1, IplImage *im2,IplImage *mask1, IplImage *mask2, int line_num);
std::vector<cv::Point> find_mask_intersections(double a, double b, double c, IplImage* mask);
#endif /* EPIPOLAR_CORRESPONDENCE_H */
