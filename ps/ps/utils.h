#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>

#include "geometric_calibration.h"

cv::Mat roto_translation(cv::Mat rot, cv::Mat trans);
cv::Mat compound_cam_transformation(cv::Mat cam, cv::Mat rot, cv::Mat trans);
double calc_norm(cv::Mat m, int size);
cv::Mat calc_essential_matrix(cv::Mat r1, cv::Mat t1, cv::Mat r2, cv::Mat t2 );
//cv::Mat calc_intersection(cv::Mat p0, cv::Mat d0, cv::Mat p1, cv::Mat d1); 

#endif /* UTILS_H */
