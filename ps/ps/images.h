#ifndef IMAGES_H
#define IMAGES_H


#include <string>
#include<vector>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>

#define IMG_HEIGHT 480
#define IMG_WIDTH 640

std::vector<IplImage*> read_images(std::string path,int cam_num,std::vector<int> &l, float scale);
IplImage* read_mask(std::string path, int cam_num, float scale);

#endif /* IMAGES_H */
