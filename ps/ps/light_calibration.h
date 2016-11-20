#ifndef LIGHT_CALIBRATION_H
#define LIGHT_CALIBRATION_H

#include <string>
#include<vector>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/common/intersections.h>

#include "geometric_calibration.h"

cv::Mat calc_light_direction(std::vector<cam_params_t> params,int cam_num, std::vector<int> availbale_lights, std::vector<double> &cam_height, int im_width, int im_h);

#endif /* LIGHT_CALIBRATION_H */
