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

int c_colors[CAM_NUM][3] = {{0,0,255},{0,255,0},{255, 0, 0},{0, 255, 255},{255, 0 ,255},{255, 255, 0},{255, 255, 255},{100, 100, 100}};

/*
void acc_votes(vector<vector<vector<vector<matchings_t>>>> &matches3D,pcl::visualization::PCLVisualizer &viewer, int h, int w) {


	vector<vector<vector<vector<vote_t>>>> votes3D(CAM_NUM,vector<vector<vector<vote_t>>>(h,vector<vector<vote_t>>(w)));
	
	
	
	vector<vote_to_pixel_t> pixel_votes;
	vote_to_pixel_t aux;

	int win_r = (int)((float)WINDOW_SIZE/2.0);
	
	int height = matches3D[0].size();
	int width = matches3D[0][0].size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
	for (int cam_idx = 0; cam_idx < CAM_NUM; cam_idx++) {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				for (int k = 0; k < matches3D[cam_idx][i][j].size(); k++) {
					cloud->points.push_back(matches3D[cam_idx][i][j][k].point);
					aux.cam_num = cam_idx;
					aux.pixel = cv::Point(j,i);
					pixel_votes.push_back(aux);
				}	
			}
		}
	}
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_ptr (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree_ptr->setInputCloud (cloud);
	std::vector<int> nIdx;
	std::vector<float> nDist;
	vote_t vote;
	for (int cam_idx = 0; cam_idx < CAM_NUM; cam_idx++) {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				vector <vote_to_pixel_t> related_pixels; 
				for (int u = -win_r; u < win_r; u++) {
					for (int v = -win_r; v < win_r; v++) {
						int x = j+v;
						int y = i+u;
						if (x < 0 || x>= width || y < 0 || y >= height) continue;
						for (int k = 0; k < matches3D[cam_idx][y][x].size(); k++) {
							vote_to_pixel_t aux;
							aux.cam_num = matches3D[cam_idx][y][x][k].paired_cam;
							aux.pixel =  matches3D[cam_idx][y][x][k].paired_pixel;
							related_pixels.push_back(aux);
						}
					}	
				}
				for (int k = 0; k < matches3D[cam_idx][i][j].size(); k++) {
					kdtree_ptr->radiusSearch(matches3D[cam_idx][i][j][k].point,VOTE_R,nIdx,nDist);
					vote.point = matches3D[cam_idx][i][j][k].point;
					vote.vote = 0;
					for (int u  = 0; u < related_pixels.size(); u++) {
						if (related_pixels[u].cam_num == matches3D[cam_idx][i][j][k].paired_cam) continue;
						for (int v = 0; v<nIdx.size(); v++) {
							if (related_pixels[u].cam_num == pixel_votes[nIdx[v]].cam_num) {
								cv::Point p_diff = related_pixels[u].pixel - pixel_votes[nIdx[v]].pixel;
								float diff = abs(p_diff.x) + abs(p_diff.y);
								if (diff <= PIX_DIST_THRESH)
									++vote.vote;
							}
						}
					}
					votes3D[cam_idx][i][j].push_back(vote);
				}
			}
		}
	}
				
	
	for (int cam_idx = 0; cam_idx < CAM_NUM; cam_idx++) {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				float max= -1;
				
				matchings_t match;
				for (int k = 0; k < votes3D[cam_idx][i][j].size(); k++) {
					if (votes3D[cam_idx][i][j][k].vote > max) {
						max = votes3D[cam_idx][i][j][k].vote;
						match.point = votes3D[cam_idx][i][j][k].point;
					}
				}
				matches3D[cam_idx][i][j].clear();
				if (max > VOTE_THRESH) {
					matches3D[cam_idx][i][j].push_back(match);	
				}
			}
		}
	}
	

	
	//viewer.addPointCloud(cloud,"all_matchings");
	//viewer.addPointCloud(sampled_cloud,"all_matchings_sampled");
	//viewer.addPointCloud(filtered_cloud,"all_matchings_filtered");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"all_matchings_sampled");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"all_matchings_sampled");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,12,"all_matchings_filtered");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"all_matchings_filtered");
	

	
	
}
*/

void acc_votes(vector<vector<vector<vector<matchings_t>>>> &matches3D,pcl::visualization::PCLVisualizer &viewer, int h, int w, vector<IplImage*> imgs) {


	
	vector<vector<vector<vector<vote_t>>>> votes3D(CAM_NUM,vector<vector<vector<vote_t>>>(h,vector<vector<vote_t>>(w)));
	
	
	
	vector<vote_to_pixel_t> pixel_votes;
	vote_to_pixel_t aux;


	
	int height = matches3D[0].size();
	int width = matches3D[0][0].size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
	for (int cam_idx = 0; cam_idx < CAM_NUM; cam_idx++) {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				for (int k = 0; k < matches3D[cam_idx][i][j].size(); k++) {
					cloud->points.push_back(matches3D[cam_idx][i][j][k].point);
					aux.cam_num = cam_idx;
					aux.pixel = cv::Point(j,i);
					aux.paired_cam = matches3D[cam_idx][i][j][k].paired_cam;
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
	
	sor.setInputCloud (cloud);
	sor.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
	sor.filter (*sampled_cloud);

	std::vector<int> nIdx;
	std::vector<float> nDist;
	vote_t v;
	for (int i = 0; i < sampled_cloud->points.size(); i++) {
		kdtree_ptr->radiusSearch(sampled_cloud->points[i],VOTE_R,nIdx,nDist);
		v.vote = nIdx.size();
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
				v.paired_pixel = pixel_votes[nIdx[j]].paired_pixel;
				votes3D[cam][y][x].push_back(v);
			}
		}
	}
	float fscale = (float)w/640.0;
	for (int cam_idx = 0; cam_idx < CAM_NUM; cam_idx++) {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				float max= -1;
				matchings_t match; 
				
				for (int k = 0; k < votes3D[cam_idx][i][j].size(); k++) {
					if (votes3D[cam_idx][i][j][k].vote > max) {
						max = votes3D[cam_idx][i][j][k].vote;
						match.point = votes3D[cam_idx][i][j][k].point;
						match.paired_cam = votes3D[cam_idx][i][j][k].paired_cam;
						match.paired_pixel = votes3D[cam_idx][i][j][k].paired_pixel;
						
					}
				}
				matches3D[cam_idx][i][j].clear();
				if (max > -1) {
					
					matches3D[cam_idx][i][j].push_back(match);	
					if (cam_idx == matches3D[cam_idx][i][j][0].paired_cam) {
							cout << "ERROR: " << cam_idx << " " << matches3D[cam_idx][i][j][0].paired_cam << endl;
							exit(0);
						}
					int x = (float)matches3D[cam_idx][i][j][0].paired_pixel.x/fscale;
					int y = (float)matches3D[cam_idx][i][j][0].paired_pixel.y/fscale;
					cvCircle(imgs[matches3D[cam_idx][i][j][0].paired_cam],cvPoint(x,y),1,cvScalar(c_colors[cam_idx][0],c_colors[cam_idx][1],c_colors[cam_idx][2]),1);
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
