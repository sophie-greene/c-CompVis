#ifndef REPRESENTATION3D_H
#define REPRESENTATION3D_H

#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>	
#include <opencv2/opencv.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/common/intersections.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/octree/octree.h>


#include "geometric_calibration.h"
#include "utils.h"

#define PIXEL3D 35

typedef struct {
	pcl::PointXYZ point;
	cv::Point paired_pixel;
	int paired_cam;
	
}matchings_t;



void add_cameras_to_world(std::vector<cam_params_t> params,  pcl::visualization::PCLVisualizer &viewer, std::string name);
void add_matchings_to_cloud(IplImage* im,pcl::visualization::PCLVisualizer &viewer,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<std::vector<cv::Point> >  points, cam_params_t params1, cam_params_t params2,int &cnt);
std::vector<std::vector<pcl::PointXYZINormal> >  add_surface_to_world(double depth_f, cv::Mat depthMap, cam_params_t params,pcl::visualization::PCLVisualizer &viewer,std::string name,int cam_num, double cam_h,IplImage* mask,pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_aux,bool print,std::vector<std::vector<cv::Mat> >  &normal_maps);
std::vector<std::vector<pcl::PointXYZINormal> >  add_surface_to_world(double depth_f, cv::Mat depthMap, cam_params_t params,pcl::visualization::PCLVisualizer &viewer,std::string name,int cam_num, double cam_h,IplImage* mask,pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_aux,bool print,std::vector<std::vector<cv::Mat> >  &normal_maps,std::vector<std::vector<std::vector<matchings_t> > > matches3D, float max_proxy_dist);
void acc_matchings(int w, int h, std::vector<std::vector<cv::Point> >  pixel_matches, std::vector<std::vector<std::vector<std::vector<matchings_t> > > >  &accumulator,cam_params_t params1, cam_params_t params2,int cam1, int cam2);
void add_matchings_to_world(std::vector<std::vector<std::vector<std::vector<matchings_t> > > >  matches3D,pcl::visualization::PCLVisualizer &viewer);
void visualize_light_dirs(std::vector<cam_params_t> params, pcl::visualization::PCLVisualizer &viewer, int cam_num, cv::Mat l) ;
void show_normals_world(std::vector<std::vector<cv::Mat> >  normals,pcl::visualization::PCLVisualizer &viewer);
void add_coloured_meshes(std::vector <IplImage*> masks, std::vector <IplImage*> ims, std::vector<std::vector<std::vector<pcl::PointXYZINormal> > > surfaces, std::vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer, float final_smooth_r,pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud,pcl::PolygonMesh hull_triangles, float leaf_size, float intersection_r);
void filter_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud);
double compute_pixel_to_real_dist(int im_w, int im_h,cam_params_t params,pcl::visualization::PCLVisualizer &viewer,std::string name,int cam_num, double &cam_height);
void avg_clouds_color(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud);
#endif /* REPRESENTATION3D_H */

