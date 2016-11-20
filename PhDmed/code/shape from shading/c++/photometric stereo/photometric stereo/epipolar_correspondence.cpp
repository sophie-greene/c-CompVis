
#include "epipolar_correspondence.h"


#define REAL_IM_WIDTH 640
#define REAL_IM_HEIGHT 480
#define LINE_RES 50

//using namespace cv;
using namespace std;


/****************************************************************************
This function calculates de intersection points of a epipolar line with the 
contour of a mask. In the normal case, these points should be 2. These 2 points
define the segment of the epipolar line that goes above the mask
*****************************************************************************/
vector<cv::Point> find_mask_intersections(double a, double b, double c, IplImage* mask) {



	bool flag = false;
	int y;
	cv::Point p1,p2;

	int im_width = mask->width;
	int im_height = mask->height;
	vector<cv::Point> intersections;

	float m = -a/b;

	//depending on the slope of the line, we sample in the x coordinate
	if (fabs(m)<1) {
		for (int x = 0; x < im_width; x++) {
			y = (-c-a*x)/ b;
			if (y < 0 || y >= im_height) continue;
			if (CV_IMAGE_ELEM(mask,uchar,y,x) > 100 && !flag) {
				p1.x = x;
				p1.y = y;
				intersections.push_back(p1);
				flag = true;
			}
			else if(CV_IMAGE_ELEM(mask,uchar,y,x) < 100 && flag) {
				p2.x = x;
				p2.y = y;
				intersections.push_back(p2);
				break;
			}
		}
	}
	else {				//or in the y coordinate
		int x;
		for (int y = 0; y < im_height; y++) {
			
			x = (-b*y-c)/a;
			
			if (x < 0 || x >= im_width) continue;
			if (CV_IMAGE_ELEM(mask,uchar,y,x) > 100 && !flag) {
				p1.x = x;
				p1.y = y;
				intersections.push_back(p1);
				flag = true;
			}
			else if(CV_IMAGE_ELEM(mask,uchar,y,x) < 100 && flag) {
				p2.x = x;
				p2.y = y;
				intersections.push_back(p2);
				break;
			}
		}
	}
	if ((intersections.size() == 2) && (intersections[0].x > intersections[1].x)) {
		cv::Point temp;
		temp = intersections[0];
		intersections[0] = intersections[1];
		intersections[1] = temp;
	}
	return intersections;
}


/****************************************************************************************************
This function calculates the corresponding epipolar line in right camera given a point in left camera.
Binary file ./PhDmed/code/shape from shading/c++/photometric stereo/photometric stereo/epipolar_correspondence.cpp matches
