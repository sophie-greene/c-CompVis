#ifndef VOTING_H
#define VOTING_H

#include <string>
#include <vector>

//#include <opencv2/imgproc/imgproc_c.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core_c.h>	
//#include <opencv2/opencv.hpp>


#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/common/intersections.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include "3Drepresentation.h"
#include "geometric_calibration.h"
#include "utils.h"

struct vote_to_pixel_t{
	int cam_num;
	cv::Point pixel;
	int paired_cam;
	cv::Point paired_pixel;
};

struct vote_t{
	float vote;
	pcl::PointXYZ point;
	cv::Point paired_pixel;
	int paired_cam;
};

void acc_votes(std::vector<std::vector<std::vector<vector<matchings_t> > > >&,pcl::visualization::PCLVisualizer &, int , int , std::vector<IplImage*> );
#endif /* VOTING_H */
