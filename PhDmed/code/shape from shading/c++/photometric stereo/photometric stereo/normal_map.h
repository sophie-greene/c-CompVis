#ifndef NORMAL_MAP_H
#define NORMAL_MAP_H

#include <string>
#include<vector>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>

#include "geometric_calibration.h"



std::vector<std::vector<cv::Mat>> calc_normal_map(std::vector<IplImage*> images, IplImage* mask_image, cv::Mat lights_dir, cv::Mat M,cv::Mat m, float low_t, float high_t, float w);

#endif /* NORMAL_MAP_H */
