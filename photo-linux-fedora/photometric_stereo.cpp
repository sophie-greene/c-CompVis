#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>




#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>

#include "3Drepresentation.h"
#include "depthMap.h"
#include "geometric_calibration.h"
#include "light_calibration.h"
#include "images.h"
#include "utils.h"
#include "normal_map.h"
#include "epipolar_correspondence.h"
#include "matching.h"

#include "voting.h"
#include "bundleAdjustment.h"
#include "groundTruth.h"

#define EPIPOLAR_SAMPLE_STEP 20

//using namespace cv;
//using namespace std;
//using namespace pcl;



pcl::visualization::PCLVisualizer viewer ("Scene with Matchings");
bool surfaceVisualizationSwitch[8] = {true,true,true,true,true,true,true,true};


//this matrix tells which camera is the one in the left when paired with other one
int cam_pairs_order[8][8] = {{0,0,2,3,4,0,0,7},
                                                         {0,1,2,1,1,5,1,7},
                                                         {2,2,2,3,4,2,2,2},
                                                         {3,1,3,3,4,3,6,3},
                                                         {4,1,4,4,4,4,6,4},
                                                         {0,5,2,3,4,5,5,7},
                                                         {0,1,2,6,6,5,6,6},
                                                         {7,7,2,3,4,7,6,7}};


float	ph_voxel_size_ ;					// size of the voxel when downsampling clouds to make the proxy hull
float	depth_constraints_weight_ ;			// weight of the absolute depth constraint in the depth map generation
float	shape_merging_iterations_;			// how many iterations do you want the cloud merging to do
float	depth_constraints_step_;			// pixel step to project intersections in the proxy hull to obtain depth constraints
float	mls_search_radius_;					// radius for smoothing the final cloud after mergin
float	voxel_decrement_;					// how much do you want the voxel size to decrease in each merging itaration
float	depth_weight_increment_;			// how much do you want the depth constrint to increase in each merging iteration
float	max_proxy_dist_;					// when calculating final point clouds for each view you do not want points further that this distance from the proxy hull
float	final_smooth_r_;
float	image_scale_factor_;
float	pixel_to_real_dist_;
float	low_normal_intens_tresh_;
float	high_normal_intens_tresh_;
float	normal_weight_;
float	final_mesh_leaf_size_;
float	intersections_search_r_;


void read_params(const char* path) {
        ifstream infile;
        std::string line;
        std::string name;
        float value;


        infile.open(path);
        if (!infile.is_open()) {
                std::cout << "Error opening file" << std::endl;
                exit(0);
        }


                while (!infile.eof()) {
                        getline(infile,line);
                        int pos = line.find(" ");
                        name = line.substr(0,pos);
                        value = atof(line.substr(pos+1,line.size()-1).c_str());


                        if (name == "ph_voxel_size_") {
                                ph_voxel_size_ = value;
                        }
                        else if (name == "depth_constraints_weight_") {
                                depth_constraints_weight_ = value;
                        }
                        else if (name == "depth_constraints_step_") {
                                depth_constraints_step_ = value;
                        }
                        else if (name == "shape_merging_iterations_") {
                                shape_merging_iterations_ = value;
                        }
                        else if (name == "mls_search_radius_") {
                                mls_search_radius_  = value;
                        }
                        else if (name == "voxel_decrement_") {
                                voxel_decrement_  = value;
                        }
                        else if (name == "depth_weight_increment_") {
                                depth_weight_increment_  = value;
                        }
                        else if (name == "max_proxy_dist_") {
                                max_proxy_dist_  = value;
                        }
                        else if (name == "final_smooth_r_") {
                                final_smooth_r_  = value;
                        }
                        else if (name == "image_scale_factor_") {
                                image_scale_factor_  = value;
                        }
                        else if (name == "pixel_to_real_dist_") {
                                pixel_to_real_dist_  = value;
                        }
                        else if (name == "low_normal_intens_tresh_") {
                                low_normal_intens_tresh_  = value;
                        }
                        else if (name == "high_normal_intens_tresh_") {
                                high_normal_intens_tresh_  = value;
                        }
                        else if (name == "normal_weight_") {
                                normal_weight_  = value;
                        }
                        else if (name == "final_mesh_leaf_size_") {
                                final_mesh_leaf_size_  = value;
                        }
                        else if (name == "intersections_search_r_") {
                                intersections_search_r_  = value;
                        }



                        std::cout << "name: " << name << " value: " << value << std::endl;
                }

}


void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*) {


 if (event.keyUp()) {

         char c = event.getKeyCode();
                if (c >= '1' && c <= '9') {
                        int num = c-49;
                        std::stringstream ss;
                        ss<<num;
                         if (surfaceVisualizationSwitch[num]) {
                                surfaceVisualizationSwitch[num] = false;
                                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0,"surface"+ss.str());

                         }
                         else {
                                 surfaceVisualizationSwitch[num] = true;
                                 viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1,"surface"+ss.str());
                         }
                }
                else if (c == 'p') {
                        viewer.removePolygonMesh("finalSurf");
                }

        }
 }


int main() {

        read_params("config.txt");
        cv::Mat laserPoints;
        std::vector<std::vector<cv::Point> >  laser_pixels = read_intrinsic_parameters("point.yml");


        viewer.registerKeyboardCallback(&keyboard_callback);


        Engine *ep;

        if (!(ep = engOpen("\0"))) {
                std::cout <<  "Can't start MATLAB engine" << std::endl;
                exit(1);
        }

        std::cout << "Matlab started" << std::endl;
        engEvalString(ep, "addpath 'D:\vs-projects\ps_imported\photometric stereo'");


        pcl::PointCloud<pcl::PointXYZ>::Ptr matchings_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::console::TicToc tt;


        //cvNamedWindow("image");
        std::vector<cam_params_t> params;
        cv::Mat light_dirs;
        std::vector<IplImage*> image_list;
        std::vector<IplImage*> image_list_gray;


        std::string calib_path = "./calib/";
        std::string data_path = "./data/";

        //float table_factor = 100;
        pcl::PointCloud<pcl::PointXYZ>::Ptr base_poly (new pcl::PointCloud<pcl::PointXYZ>);
        base_poly->points.push_back(pcl::PointXYZ(-20,-20,0));
        base_poly->points.push_back(pcl::PointXYZ(-20,20,0));
        base_poly->points.push_back(pcl::PointXYZ(20,20,0));
        base_poly->points.push_back(pcl::PointXYZ(20,-20,0));
        viewer.addPolygon<pcl::PointXYZ>(base_poly,1,0,0,"table");
        /*
        pcl::PointCloud<pcl::PointXYZ>::Ptr heart_poly (new pcl::PointCloud<PointXYZ>);
        heart_poly->points.push_back(PointXYZ(10,0,0));
        heart_poly->points.push_back(PointXYZ(17,0,0));
        heart_poly->points.push_back(PointXYZ(17,12,0));
        heart_poly->points.push_back(PointXYZ(10,12,0));
        viewer.addPolygon<PointXYZ>(heart_poly,0,1,0,"heart_size1");

        pcl::PointCloud<pcl::PointXYZ>::Ptr heart_poly2 (new pcl::PointCloud<PointXYZ>);
        heart_poly2->points.push_back(PointXYZ(17,0,0));
        heart_poly2->points.push_back(PointXYZ(17,0,8));
        heart_poly2->points.push_back(PointXYZ(17,12,8));
        heart_poly2->points.push_back(PointXYZ(17,12,0));
        viewer.addPolygon<PointXYZ>(heart_poly2,0,1,0,"heart_size2");
        */
        params  = read_cameras_calibration(calib_path);
        viewer.addCoordinateSystem(2,0,0,0);





        add_cameras_to_world(params,viewer,"1");


        /*
        for(int i = 0; i < 8; i++) {
                cvShowImage("image", image_list[i]);
                cvWaitKey();
        }
        cvShowImage("image", mask_image);
        cvWaitKey();
        */

        //test(laser_pixels);
        //exit(0);


        /*
        while (1) {
                viewer.spinOnce ();
                pcl_sleep(0.1);
        }
        */


        //normal maps calculation
        std::vector<std::vector<std::vector<cv::Mat> > > normal_map_list;

        std::vector<IplImage*> images;
        std::vector<IplImage*> masks;

        std::vector<double> cameras_height;
        std::vector<double> cam_pixel_dist_factors;

        //int image_trad[] = {6,1,6,3,3,6,6,6};

        //int image_trad[] = {5,0,4,2,2,0,0,6};
        for (int i = 0; i < CAM_NUM; i++) {
                std::vector<int> available_lights;
                image_list = read_images(data_path,i,available_lights,image_scale_factor_);
                image_list_gray = std::vector<IplImage*>(image_list.size());
                for (int j = 0; j < image_list.size(); j++) {
                        image_list_gray[j] = cvCreateImage(cvSize(image_list[j]->width,image_list[j]->height),IPL_DEPTH_8U,1);
                        cvCvtColor(image_list[j],image_list_gray[j],CV_BGR2GRAY);
                }
                light_dirs = calc_light_direction(params,i,available_lights,cameras_height, image_list_gray[0]->width,image_list_gray[0]->height);
                IplImage* mask_image = read_mask(data_path, i,image_scale_factor_);

                //images.push_back(image_list[image_trad[i]]);
                images.push_back(image_list[i]);
                masks.push_back(mask_image);

                //visualize_light_dirs(params, viewer, i, light_dirs);
                //cvWaitKey();
                cv::Mat M = roto_translation(params[i].rot,params[i].trans);

                tt.tic();

                std::vector<std::vector<cv::Mat> >  normals = calc_normal_map(image_list_gray, mask_image,light_dirs,M,params[i].trans,low_normal_intens_tresh_,high_normal_intens_tresh_,normal_weight_);
                std::cout << "Normals [done]: " <<tt.toc() << " ms"<<std::endl;
                //show_normals_world(normals,viewer);

                /*
                std::stringstream ss;
                ss<<i;

                visualize_normals_world(normals,params[i],"cam"+ss.str(),"world"+ss.str());
                */

                /*
                Mat depthMap = calc_depthMap(normals, mask_image,ep);

                cvWaitKey();
                tt.tic();
                add_surface_to_world(depthMap, params[i],viewer, "surface"+ss.str(),i);
                std::cout << "Depth Map [done]: " <<tt.toc() << " ms"<<std::endl;
                */

                normal_map_list.push_back(normals);
                //cam_pixel_dist_factors.push_back( compute_pixel_to_real_dist(image_list_gray[0]->width, image_list_gray[0]->height,params[i],viewer,"",i,cameras_height[i]));
        }


        //engClose(ep);
        /*while (1) {
                viewer.spinOnce ();
                pcl_sleep(0.1);
        }
        exit(0);
        */
        // epipolar correspondences and matchings
        cv::Mat E;
        int im_height = images[0]->height;
        int im_width = images[0]->width;
        std::vector<std::vector<cv::Point> >  ep_lines;
        std::vector<std::vector<double> >  matching_cost;
        //for all camera pairs
        //Mat M = roto_translation(params[7].rot,params[7].trans);
        //visualize_normals_world(normal_map_list[7], M,params[7].pos);
        //cvNamedWindow("Left Image");
        //cvNamedWindow("Right Image");
        IplImage* left_im = cvCreateImage(cvSize(im_width,im_height),IPL_DEPTH_8U,3);
        IplImage* right_im = cvCreateImage(cvSize(im_width,im_height),IPL_DEPTH_8U,3);

        IplImage* left_im_big = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
        IplImage* right_im_big = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);

        IplImage* undistorted_im = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);


        std::vector<std::vector<cv::Point> >  pixel_matches;
        std::vector<std::vector<std::vector<std::vector<matchings_t> > > >  matches3D(CAM_NUM,std::vector<std::vector<std::vector<matchings_t> > >(im_height,std::vector<std::vector<matchings_t> > (im_width)));

        std::vector<IplImage*> retropro(CAM_NUM);
        for (int i = 0; i < CAM_NUM; i++) {
                IplImage* aux = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
                retropro[i] = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);

                cvResize(images[i],aux);

                cvCvtColor(aux,retropro[i],CV_BGR2RGB);
        }

        for (int i = 0; i < CAM_NUM; i++) {
                for (int j = 0; j < CAM_NUM; j++) {
                        if (i==j) continue;

                        int left_cam = cam_pairs_order[i][j];
                        int right_cam;
                        if (i == left_cam) right_cam = j;
                        else right_cam = i;

                        left_im= visualize_normals_world ( normal_map_list[left_cam], params[left_cam],"","");
                        right_im= visualize_normals_world ( normal_map_list[right_cam], params[right_cam],"","");
                        cvResize(left_im,left_im_big);
                        cvResize(right_im,right_im_big);


                        E = calc_essential_matrix(params[left_cam].rot,params[left_cam].trans,params[right_cam].rot,params[right_cam].trans);

                        //for certain points in each camera frame compute epipolar correspondence
                        int line_cnt = 0;
                        for (int v = 0; v < im_height; v+=EPIPOLAR_SAMPLE_STEP) {
                                for (int u = 0; u < im_width; u+=EPIPOLAR_SAMPLE_STEP) {

                                        //int u = 404;
                                        //int v = 76;

                                        if (CV_IMAGE_ELEM(masks[left_cam],uchar,v,u) < 100) continue;
                                        //cvCircle(left_im,cvPoint(u,v),2,cvScalar(0,0,255),2);
                                        //std::cout << u << " " << v << std::endl;
                                        //tt.tic();
                                                ++line_cnt;
                                        ep_lines = find_epipolar_lines(E,u,v,params[left_cam].cam,params[right_cam].cam,left_im_big,right_im_big,masks[left_cam],masks[right_cam],line_cnt);
                                        //std::cout << "Epipolar correspondences [done]: " <<tt.toc() << " ms"<<std::endl;
                                        if (ep_lines[0].size() == 0 || ep_lines[1].size() == 0 ) continue;
                                        //tt.tic();

                                        //matching_cost = calc_matching_cost(ep_lines, normal_map_list[i],normal_map_list[j],params[i],params[j]);
                                        matching_cost = calc_matching_cost(ep_lines, left_im,right_im);
                                        //std::cout << "Matching cost matrix [done]: " <<tt.toc() << " ms"<<std::endl;
                                        //visualize_matching_cost(matching_cost);
                                        //tt.tic();
                                        pixel_matches = find_best_path(matching_cost,ep_lines, left_im_big, right_im_big,im_width,im_height);
                                        //std::cout << "Best path  [done]: " <<tt.toc() << " ms"<<std::endl;

                                        acc_matchings(im_width,im_height, pixel_matches,matches3D,params[left_cam],params[right_cam],left_cam,right_cam);

                                        //std::cout << "Accumulating matchings" << std::endl;

                                        //add_matchings_to_cloud(left_im,viewer,matchings_cloud, pixel_matches,params[i], params[j],line_cnt);
                                //	cvWaitKey();


                                //	cvShowImage("Left image",left_im_big);
                                //	cvShowImage("Right image",right_im_big);

                                //	cvWaitKey();



                                }
                        }

                        //cv::Mat depthMap_left = calc_depthMap(normal_map_list[left_cam], masks[left_cam],ep,matches3D[left_cam],params[left_cam],cameras_height[left_cam]);
                        //cv::Mat depthMap_right = calc_depthMap(normal_map_list[right_cam], masks[right_cam],ep,matches3D[right_cam],params[right_cam],cameras_height[right_cam]);
                        //cvWaitKey();

                        /*
                        tt.tic();
                        std::stringstream ss;
                        ss<<left_cam;
                        add_surface_to_world(depthMap_left, params[left_cam],viewer, "surface"+ss.str(),left_cam, cameras_height[left_cam]);
                        std::stringstream sss;
                        sss<<right_cam;
                        add_surface_to_world(depthMap_right, params[right_cam],viewer, "surface"+sss.str(),right_cam,cameras_height[right_cam]);
                        std::cout << "Depth Map [done]: " <<tt.toc() << " ms"<<std::endl;
                        */
                }
        }
        acc_votes(matches3D,viewer,im_height,im_width,retropro);
        //add_matchings_to_world(matches3D,viewer);
        //cvWaitKey();

        std::vector<std::vector<std::vector<pcl::PointXYZINormal> > > surfaces(CAM_NUM);

        double   depth_weight = 0.0005;

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_aux (new pcl::PointCloud<pcl::PointXYZINormal>);
        std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>  clouds_aux;
        for (int i = 0; i < CAM_NUM; i++) {
                //if (i != 4 && i != 7) continue;
                cv::Mat depthMap = calc_depthMap(pixel_to_real_dist_ ,normal_map_list[i], masks[i],ep,matches3D[i],params[i],cameras_height[i],viewer,depth_weight);
                std::stringstream ss;
                ss<<i;
                //compute_pixel_to_real_dist(depthMap,params[i],viewer,"surface"+ss.str(),i,cameras_height[i]);

                surfaces[i] = add_surface_to_world(pixel_to_real_dist_,depthMap, params[i],viewer, "surface"+ss.str(),i, cameras_height[i],masks[i], cloud_aux,0,normal_map_list[i]);
                clouds_aux.push_back(cloud_aux);

                //cvWaitKey();
        }


        depth_weight = depth_constraints_weight_;
        float voxel_size = ph_voxel_size_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_proxy_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PolygonMesh final_triangles;
        std::vector<cv::Mat> depthMap_list;
        for (int iter = 0; iter < shape_merging_iterations_; iter ++) {
                std::cout << "iter " << iter << std::endl;
                pcl::PolygonMesh triangles;
                pcl::PointCloud<pcl::PointXYZ>::Ptr proxy_hull(new pcl::PointCloud<pcl::PointXYZ>);

                create_avg_surface(clouds_aux,viewer, triangles, proxy_hull, voxel_size,mls_search_radius_);
                matches3D = proxy_hull_ray_intersections(proxy_hull, surfaces,params,viewer, triangles,depth_constraints_step_);
                clouds_aux.clear();
                depthMap_list.clear();
                for (int i = 0; i < CAM_NUM; i++) {
                        //if (i != 4 && i != 7) continue;

                        cv::Mat depthMap = calc_depthMap(pixel_to_real_dist_ ,normal_map_list[i], masks[i],ep,matches3D[i],params[i],cameras_height[i],viewer,depth_weight);
                        depthMap_list.push_back(depthMap);
                        std::stringstream ss;
                        ss<<i;
                        surfaces[i] = add_surface_to_world(pixel_to_real_dist_ ,depthMap, params[i],viewer, "surface"+ss.str(),i, cameras_height[i],masks[i], cloud_aux,0,normal_map_list[i]);
                        clouds_aux.push_back(cloud_aux);


                }
                //cvWaitKey();
                voxel_size  = voxel_size  - voxel_decrement_;
                if (voxel_size < 1) voxel_size = 1;
                std::cout << "voxel decrement: "  << voxel_decrement_  << std::endl;
                depth_weight = depth_weight+ depth_weight_increment_;
                final_proxy_hull = proxy_hull;
                final_triangles = triangles;
        }

        std::cout << "final intersections for filtering" << std::endl;
        matches3D = proxy_hull_ray_intersections(final_proxy_hull, surfaces,params,viewer, final_triangles,1);
        for (int i = 0; i < CAM_NUM; i++) {
                std::stringstream ss;
                ss<<i;
                surfaces[i] = add_surface_to_world(pixel_to_real_dist_ ,depthMap_list[i], params[i],viewer, "surface"+ss.str(),i, cameras_height[i],masks[i], cloud_aux,0,normal_map_list[i], matches3D[i],max_proxy_dist_);
        }

        std::vector<IplImage*> full_res_ims(CAM_NUM);
        std::vector<IplImage*> full_res_masks(CAM_NUM);

        full_res_ims[0] = cvLoadImage("./data_7/cam0light2.bmp");
        full_res_ims[1] = cvLoadImage("./data_7/cam1light1.bmp");
        full_res_ims[2] = cvLoadImage("./data/cam2light2.bmp");
        full_res_ims[3] = cvLoadImage("./data_7/cam3light1.bmp");
        full_res_ims[4] = cvLoadImage("./data_7/cam4light1.bmp");
        full_res_ims[5] = cvLoadImage("./data_7/cam5light1.bmp");
        full_res_ims[6] = cvLoadImage("./data_7/cam6light1.bmp");
        full_res_ims[7] = cvLoadImage("./data_7/cam7light1.bmp");

        full_res_masks[0] = cvLoadImage("./data/cam0mask.jpg",0);
        full_res_masks[1] = cvLoadImage("./data/cam1mask.jpg",0);
        full_res_masks[2] = cvLoadImage("./data/cam2mask.jpg",0);
        full_res_masks[3] = cvLoadImage("./data/cam3mask.jpg",0);
        full_res_masks[4] = cvLoadImage("./data/cam4mask.jpg",0);
        full_res_masks[5] = cvLoadImage("./data/cam5mask.jpg",0);
        full_res_masks[6] = cvLoadImage("./data/cam6mask.jpg",0);
        full_res_masks[7] = cvLoadImage("./data/cam7mask.jpg",0);


        add_coloured_meshes(full_res_masks,full_res_ims, surfaces,params,viewer,final_smooth_r_,final_proxy_hull,final_triangles, final_mesh_leaf_size_,intersections_search_r_);

        //build_ground_truth_cloud(viewer, params, laser_pixels);





        while (1) {
                viewer.spinOnce ();
                pcl_sleep(0.1);
        }




}
