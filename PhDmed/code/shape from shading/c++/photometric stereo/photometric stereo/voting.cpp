#include "voting.h"

#define VOXEL_SIZE 2
#define VOTE_R 1
#define PIX_DIST_THRESH 10
#define VOTE_THRESH 500
#define WINDOW_SIZE 10

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

// colors asociated to each camera for plotting 

int c_colors[CAM_NUM][3] = {{0,0,255},{0,255,0},{255, 0, 0},{0, 255, 255},{255, 0 ,255},{255, 255, 0},{255, 255, 255},{100, 100, 100}};

/******************************************************************
Subdivide the space and count how many matches lie in each zone. 

Input:
	matches3D: are the 3D points where pixel matches have been found in the epipolar correspondence pairing cameras.
	viewer: is the viewer just for plotting purposes if you want to plot anything
	h and w: are height and width of images
	images: a vector containing tha images.

*******************************************************************/
void acc_votes(vector<vector<vector<vector<matchings_t> > > >  &matches3D,pcl::visualization::PCLVisualizer &viewer, int h, int w, vector<IplImage*> imgs) {


	vector<vector<vector<vector<vote_t> > > >  votes3D(CAM_NUM,vector<vector<vector<vote_t> > >(h,vector<vector<vote_t> > (w)));
		
	vector<vote_to_pixel_t> pixel_votes;
	vote_to_pixel_t aux;


	
	int height = matches3D[0].size();
	int width = matches3D[0][0].size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
	//loop that iterates for eveery camera
	for (int cam_idx = 0; cam_idx < CAM_NUM; cam_idx++) {
		//loop that iterates for every pixel in each camera
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				//loop that iterates for each matching in each pixel
				for (int k = 0; k < matches3D[cam_idx][i][j].size(); k++) {
					
					//all the matches are accumulated into a single point cloud
					cloud->points.push_back(matches3D[cam_idx][i][j][k].point);

					//each matching is saved into a voting type in wich we have the cam that is casting the vote and the pixel of this cam,
					//the paired pixel and the paired camera of the matching
					aux.cam_num = cam_idx;
					aux.pixel = cv::Point(j,i);
					aux.paired_cam = matches3D[cam_idx][i][j][k].paired_cam;
					//just to check, a camera cannot be paired with itself. If this happens it is because something went wrong 
					if (aux.paired_cam == cam_idx) {
						cout << "ERROR: " << cam_idx << " " << aux.paired_cam << endl;
						exit(0);
					}
					aux.paired_pixel =matches3D[cam_idx][i][j][k].paired_pixel; 
					pixel_votes.push_back(aux);
				}	
			}
		}
	}
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_ptr (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree_ptr->setInputCloud (cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud (new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	
	//the cloud containing all the matches is downsampled. The points of the downsampled cloud are going to be used as
	//the centers of spheres. The spheres are zones in which the matches are counted 
	sor.setInputCloud (cloud);
	sor.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
	sor.filter (*sampled_cloud);

	std::vector<int> nIdx;
	std::vector<float> nDist;
	vote_t v;
	//for every point of the sampled point cloud
	for (int i = 0; i < sampled_cloud->points.size(); i++) {
		//search for neighbours in a radius
		kdtree_ptr->radiusSearch(sampled_cloud->points[i],VOTE_R,nIdx,nDist);
		//the value of the vote is the amount of neighbours in this volume
		v.vote = nIdx.size();
		//if there is a sufficient amount of neighbours in this volume
		if (v.vote > VOTE_THRESH) {
			filtered_cloud->points.push_back(sampled_cloud->points[i]);
			for (int j = 0; j < nIdx.size(); j++) {
				v.point = cloud->points[nIdx[j]];
				int cam = pixel_votes[nIdx[j]].cam_num;
				int x = pixel_votes[nIdx[j]].pixel.x;
				int y = pixel_votes[nIdx[j]].pixel.y;
				v.paired_cam = pixel_votes[nIdx[j]].paired_cam;
				if (cam == v.paired_cam) {
						cout << "ERROR: " << cam << " " << v.paired_cam << endl;
						exit(0);
					}
				//for every neighbour, look to wich camera and pixel correspond this point and accumulate the vote there
				v.paired_pixel = pixel_votes[nIdx[j]].paired_pixel;
				votes3D[cam][y][x].push_back(v);
			}
		}
	}
	float fscale = (float)w/640.0;
	//iterate through all the cameras and pixels
	for (int cam_idx = 0; cam_idx < CAM_NUM; cam_idx++) {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				float max= -1;
				matchings_t match; 
				//iterate throgh all the votes (a.k.a possible 3D positions where this pixel may be)
				//and select the maximum vote
				for (int k = 0; k < votes3D[cam_idx][i][j].size(); k++) {
					if (votes3D[cam_idx][i][j][k].vote > max) {
						max = votes3D[cam_idx][i][j][k].vote;
						match.point = votes3D[cam_idx][i][j][k].point;
						match.paired_cam = votes3D[cam_idx][i][j][k].paired_cam;
						match.paired_pixel = votes3D[cam_idx][i][j][k].paired_pixel;
						
					}
				}
				//erase all information of matches3D, we are going to use this variable to include the maximum voted position for each pixel
				matches3D[cam_idx][i][j].clear();
				//if there is a maximum in this pixel
				if (max > -1) {
					//include the position of the maximum vote as the new position of the pixel 
					matches3D[cam_idx][i][j].push_back(match);	
					if (cam_idx == matches3D[cam_idx][i][j][0].paired_cam) {
							cout << "ERROR: " << cam_idx << " " << matches3D[cam_idx][i][j][0].paired_cam << endl;
							exit(0);
						}
					//this is just for visualization if you uncomment below
					
					//int x = (float)matches3D[cam_idx][i][j][0].paired_pixel.x/fscale;
					//int y = (float)matches3D[cam_idx][i][j][0].paired_pixel.y/fscale;
					//cvCircle(imgs[matches3D[cam_idx][i][j][0].paired_cam],cvPoint(x,y),1,cvScalar(c_colors[cam_idx][0],c_colors[cam_idx][1],c_colors[cam_idx][2]),1);
				}
			}
		}
	}
	
	/*
	for (int i = 0; i < imgs.size(); i++) {
		stringstream ss;
		ss << i;
		cvNamedWindow(ss.str().c_str());

		cvShowImage(ss.str().c_str(),imgs[i]);
	}
	*/
	
	//viewer.addPointCloud(cloud,"all_matchings");
	//viewer.addPointCloud(sampled_cloud,"all_matchings_sampled");
	//viewer.addPointCloud(filtered_cloud,"all_matchings_filtered");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"all_matchings_sampled");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"all_matchings_sampled");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,12,"all_matchings_filtered");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"all_matchings_filtered");
	//cvWaitKey();

	
	
}
