#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>




#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/common/intersections.h>

#include "geometric_calibration.h"

#include "utils.h"

std::vector<std::vector<cv::Point> > read_intrinsic_parameters(std::string file_name/*, cv::Mat &laserPoints*/);
void build_ground_truth_cloud(pcl::visualization::PCLVisualizer &viewer, std::vector<cam_params_t> params, std::vector<std::vector<cv::Point> > laser_pixels);
void test(std::vector<std::vector<cv::Point> > laser_pixels);
#endif /* GROUNDTRUTH_H */
