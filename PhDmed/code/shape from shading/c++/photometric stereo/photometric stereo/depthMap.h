#ifndef	DEPTHMAP_H	
#define DEPTHMAP_H
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>

#include <Eigen/Sparse>


#include <engine.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/common/intersections.h>

#include "geometric_calibration.h"
#include "utils.h"
#include "3Drepresentation.h"






cv::Mat calc_depthMap(double depth_factor, std::vector<std::vector<cv::Mat> >  normals, IplImage* mask,Engine *ep,std::vector<std::vector<std::vector<matchings_t> > > points3D, cam_params_t params, double cam_h,pcl::visualization::PCLVisualizer &viewer, double weight);
#endif /* DEPTHMAP_H */
