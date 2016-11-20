#ifndef	BUNDLEADJUSTMENT_H	
#define BUNDLEADJUSTMENT_H

#include <pcl/features/normal_3d.h>
#include <string>
#include <vector>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>

//#include <Eigen/Sparse>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>


#include <pcl/registration/transforms.h>
#include <engine.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>



#include <pcl/common/intersections.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include "geometric_calibration.h"
#include "utils.h"
#include "3Drepresentation.h"

typedef struct {
	float dist;
	int index;
} avg_dist_t;

pcl::PointXYZ bundle_adjustment(std::vector<std::vector<pcl::PointXYZ>> surface, int im_w, int im_h, cam_params_t par1, cam_params_t par2,pcl::visualization::PCLVisualizer &viewer, IplImage* im1, IplImage* im2,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux);
void re_calibrate_cams(std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds, std::vector<cam_params_t> &params,pcl::visualization::PCLVisualizer &viewer);
void prueba(std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clouds,pcl::visualization::PCLVisualizer &viewer);
 std::vector<std::vector<std::vector<std::vector<matchings_t>>>> refine_depth_constraints(std::vector<std::vector<std::vector<cv::Mat>>> &normal_maps,std::vector<std::vector<std::vector<pcl::PointXYZINormal>>> surfaces, std::vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer);
 std::vector<std::vector<std::vector<std::vector<matchings_t>>>> refine_depth_constraints_one_cam(int cam_idx,std::vector<std::vector<std::vector<cv::Mat>>> &normal_maps,std::vector<std::vector<std::vector<pcl::PointXYZINormal>>> surfaces, std::vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer);
 void deform_clouds(std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>& clouds, std::vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer);
void smoothing(std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds, std::vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer);
void create_avg_surface(std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clouds,pcl::visualization::PCLVisualizer &viewer, pcl::PolygonMesh &triangles,pcl::PointCloud<pcl::PointXYZ>::Ptr &mls_points, float voxel_size,float,float,float,float,float);
std::vector<std::vector<std::vector<std::vector<matchings_t>>>> proxy_hull_ray_intersections( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<std::vector<std::vector<pcl::PointXYZINormal>>> surfaces, std::vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer, pcl::PolygonMesh triangles,float density);
void filter_surface_proxy_hull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector<std::vector<std::vector<pcl::PointXYZINormal>>> &surfaces, std::vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer, pcl::PolygonMesh triangles, float ray_step);
#endif /* BUNDLEADJUSTMENT_H */
