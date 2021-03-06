
#include "bundleAdjustment.h"
using namespace std;
using namespace pcl;
//using namespace cv;

#define SAMPLE_SIZE 2
#define X_RANGE 1
#define Y_RANGE 1
#define Z_RANGE 1
#define CALIB_STEP 1
#define RX_RANGE 0.15
#define RY_RANGE 0.15
#define RZ_RANGE 0.15
#define CALIB_STEP_R 0.05
#define NORMAL_DIST_W 100.0
#define CALIB_THRESH 0.40

#define REFIN_SEARCH_R 8
#define VOXEL_SIZE 5
#define DIRECTED_NEIGH_RADIAL_DIST 0.5

int cam_col[CAM_NUM][3] = {{255,0,0},{0,255,0},{0, 0, 255},{255, 255, 0},{255, 0 ,255},{0, 255, 255},{255, 255, 255},{100, 100, 100}};
/*
PointXYZ bundle_adjustment(vector<vector<PointXYZ>> surface, int im_w, int im_h, cam_params_t par1, cam_params_t par2,pcl::visualization::PCLVisualizer &viewer, IplImage* im1, IplImage* im2, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux) {

	float fscale = (float)im_w/640.0;
	
	vector<Point2d> pixels_cam1;
	vector<Point2d> pixels_cam2;
	vector<Point2d> reprojected_pixels;
	// variables for sba
	std::vector<cv::Point3d>                points_true, points_init, points_opt;
	std::vector<std::vector<cv::Point2d> >  imagePoints(2);
	std::vector<std::vector<int> >          visiblity(2,vector<int>(1,1));
	std::vector<cv::Mat>                    cam_mats, cam_mats_copy;
	std::vector<cv::Mat>                    R_mats, R_mats_copy;
	std::vector<cv::Mat>                    T_vecs, T_vecs_copy;
	std::vector<cv::Mat>                    distCoeffs, empty_dist(2);
	cv::TermCriteria                        criteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 300, 1e-10);
	
	cam_mats.push_back(par1.cam);
	cam_mats.push_back(par2.cam);

	cam_mats_copy.push_back(par1.cam);
	cam_mats_copy.push_back(par2.cam);

	R_mats.push_back(par1.rot);
	R_mats.push_back(par2.rot);
	
	R_mats_copy.push_back(par1.rot);
	R_mats_copy.push_back(par2.rot);
	
	
	T_vecs.push_back(par1.trans);
	T_vecs.push_back(par2.trans);
	
	T_vecs_copy.push_back(par1.trans);
	T_vecs_copy.push_back(par2.trans);
	
	
	distCoeffs.push_back(par1.dist);
	distCoeffs.push_back(par2.dist);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<PointXYZ>);
	//cout << "Distorsion: "<<par1.dist << endl;

	Point2d p;
	
	p = cvPoint(417,225); pixels_cam1.push_back(p);
	
	p = cvPoint(410,266); pixels_cam1.push_back(p);
	p = cvPoint(489,185); pixels_cam1.push_back(p);
	p = cvPoint(474,269); pixels_cam1.push_back(p);
	p = cvPoint(527,240); pixels_cam1.push_back(p);
	p = cvPoint(460,287); pixels_cam1.push_back(p);
	p = cvPoint(451,167); pixels_cam1.push_back(p);
	p = cvPoint(410,170); pixels_cam1.push_back(p);
	p = cvPoint(477,309); pixels_cam1.push_back(p);
	p = cvPoint(532,225); pixels_cam1.push_back(p);
	p = cvPoint(384,201); pixels_cam1.push_back(p);
	
	

	imagePoints[0] = pixels_cam1;

	
	p = cvPoint(226,254); pixels_cam2.push_back(p);
	
	p = cvPoint(191,259); pixels_cam2.push_back(p);
	
	p = cvPoint(303,352); pixels_cam2.push_back(p);
	p = cvPoint(179,310); pixels_cam2.push_back(p);
	p = cvPoint(219,360); pixels_cam2.push_back(p);
	p = cvPoint(170,304); pixels_cam2.push_back(p);
	p = cvPoint(324,308); pixels_cam2.push_back(p);
	p = cvPoint(316,259); pixels_cam2.push_back(p);
	p = cvPoint(156,331); pixels_cam2.push_back(p);
	p = cvPoint(270,396); pixels_cam2.push_back(p);
	p = cvPoint(260,228); pixels_cam2.push_back(p);
	
	
	
	imagePoints[1] = pixels_cam2;
	
	for (int i= 0; i < pixels_cam2.size(); i++) {
		
		int x = (float)(pixels_cam2[i].x)*fscale;
		int y = (float)(pixels_cam2[i].y)*fscale;
		
		Point3d p3d;
		p3d.x = surface[y][x].x;
		p3d.y = surface[y][x].y;
		p3d.z = surface[y][x].z;
		points_true.push_back(p3d);

		cloud->points.push_back(surface[y][x]);


	}
	

	cv::projectPoints(points_true, R_mats[0], T_vecs[0], cam_mats[0], distCoeffs[0], reprojected_pixels);
	//imagePoints[0] = reprojected_pixels;
	
	cv::projectPoints(points_true, R_mats[1], T_vecs[1], cam_mats[1], distCoeffs[1], reprojected_pixels);
	imagePoints[1] = reprojected_pixels;
	cvNamedWindow("reprojection1");
	cvNamedWindow("reprojection2");
	for (int i = 0; i < reprojected_pixels.size(); i++) {
		cvCircle(im2,reprojected_pixels[i],2,cvScalar(0,0,255),2);
		cvCircle(im1,imagePoints[0][i],2,cvScalar(0,0,255),2);
	}
	
	Mat pos = -R_mats[1].t() * T_vecs[1];

	PointXYZ final_pos;
	final_pos.x = pos.at<double>(0);
	final_pos.y = pos.at<double>(1);
	final_pos.z = pos.at<double>(2);

	
	std::vector<cv::Mat> distCoeffs_aux = distCoeffs;
	
	// run bundle adjustment
	
	//cv::LevMarqSparse   lms;
	
	//cout << "antes" << endl;
	//lms.bundleAdjust(points_true, imagePoints, visiblity, cam_mats, R_mats, T_vecs, empty_dist, criteria);
	//cout << "despues" << endl;
	
	cv::solvePnP(points_true,imagePoints[0],cam_mats[0],distCoeffs[0],R_mats[0],T_vecs[0]);
	cv::Rodrigues(R_mats[0], R_mats[0]);
	for (int i = 0; i < points_true.size(); i++) {
		PointXYZ p;
		p.x = points_true[i].x;
		p.y = points_true[i].y;
		p.z = points_true[i].z;
		cloud_2->points.push_back(p);
		
	}

	//cout << "rot after: " << R_mats[1] <<endl
	//	<< "trans after: " << T_vecs[1] << endl;

	//cout << "dist cam1 before: " << distCoeffs_aux[0] <<endl
		// << "dist cam1 after: " << empty_dist[0] <<endl
		// << "dist cam2 before: " << distCoeffs_aux[1] <<endl;
		// << "dist cam2 after: " << empty_dist[1] <<endl;

	cout << "cam1 before: " << cam_mats_copy[0] <<endl
		 << "cam1 after: " << cam_mats[0] <<endl;
	cout << "cam2 before: " << cam_mats_copy[1] <<endl
		 << "cam2 after: " << cam_mats[1] <<endl;
	
	cout << "rot1 before: " << R_mats_copy[0] <<endl
		 << "rot1 after: " << R_mats[0] <<endl;
	cout << "rot2 before: " << R_mats_copy[1] <<endl
		 << "rot2 after: " << R_mats[1] <<endl;
	
	cout << "trans1 before: " << T_vecs_copy[0] <<endl
		 << "trans1 after: " << T_vecs[0] <<endl;
	cout << "trans2 before: " << T_vecs_copy[1] <<endl
		 << "trans2 after: " << T_vecs[1] <<endl;
	
	cv::projectPoints(points_true, R_mats[0], T_vecs[0], cam_mats[0], distCoeffs[0], reprojected_pixels);
	
	
	for (int i = 0; i < reprojected_pixels.size(); i++) {
		cvCircle(im1,reprojected_pixels[i],2,cvScalar(0,255,0),2);
		cout << "or: " << imagePoints[0][i] << endl;
		cout << "pr: " << reprojected_pixels[i] << endl;

	}

	cv::projectPoints(points_true, R_mats[1], T_vecs[1], cam_mats[1], distCoeffs[1], reprojected_pixels);
	cout << "-----------------------" << endl;
	for (int i = 0; i < reprojected_pixels.size(); i++) {
		cvCircle(im2,reprojected_pixels[i],2,cvScalar(0,255,0),2);
		cout << "or: " << imagePoints[1][i] << endl;
		cout << "pr: " << reprojected_pixels[i] << endl;

	}
	
	
	cvShowImage("reprojection1",im1);
	cvShowImage("reprojection2",im2);

	cout << "Hola " << endl;
	cv::Mat       Rw, Tw;
	Eigen::Matrix4f   _t,_t2;
    Eigen::Affine3f   t,t2;

	Rw =  R_mats[0].t();
    Tw = -Rw*T_vecs[0];

	_t << Rw.at<double>(0,0), Rw.at<double>(0,1), Rw.at<double>(0,2), Tw.at<double>(0,0),
          Rw.at<double>(1,0), Rw.at<double>(1,1), Rw.at<double>(1,2), Tw.at<double>(1,0),
          Rw.at<double>(2,0), Rw.at<double>(2,1), Rw.at<double>(2,2), Tw.at<double>(2,0),
          0.0, 0.0, 0.0, 1.0;
    
    t = _t;
	viewer.addCoordinateSystem(2.0, t);

	cv::Mat rot_vec(3,1,CV_64F);
	rot_vec.at<double>(0) = 0;//CV_PI/4.0;
	rot_vec.at<double>(1) = 0;
	rot_vec.at<double>(2) = 0;
	cv::Rodrigues(rot_vec,R_mats[0]);
    

	_t2 << R_mats[0].at<double>(0,0), R_mats[0].at<double>(0,1), R_mats[0].at<double>(0,2), 10,
          R_mats[0].at<double>(1,0), R_mats[0].at<double>(1,1), R_mats[0].at<double>(1,2), 0,
          R_mats[0].at<double>(2,0), R_mats[0].at<double>(2,1), R_mats[0].at<double>(2,2), 0,
          0.0, 0.0, 0.0, 1.0;
    t2 = _t2;
	 
	
	
	viewer.addCoordinateSystem(2.0, t*t2*t.inverse());


	pcl::PointCloud<pcl::PointXYZ> cloud_out;
	pcl::PointCloud<pcl::PointXYZ> cloud_in;

	cout << "Hola1" << endl;
	for (int i = 0; i< cloud_aux->points.size(); i++) {
		cloud_in.points.push_back(cloud_aux->points[i]);
	}
	cout << "Hola2" << endl;

	Eigen::Affine3f final = t*t2*t.inverse();
	cout << "Hola3" << endl;
	
	pcl::transformPointCloud(cloud_in,cloud_out,final);	
	cout << "Hola4" << endl;
	cloud_aux->points.clear();
	cout << "Hola5" << endl;
	for (int i = 0; i< cloud_out.points.size(); i++) {
		cloud_aux->points.push_back(cloud_out.points[i]);
	}

	cout << "Hola6" << endl;
	viewer.addPointCloud(cloud_aux,"transformed");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"transformed");
	
	Rw =  R_mats[1].t();
	 Tw = -Rw*T_vecs[1];

	_t << Rw.at<double>(0,0), Rw.at<double>(0,1), Rw.at<double>(0,2), Tw.at<double>(0,0),
          Rw.at<double>(1,0), Rw.at<double>(1,1), Rw.at<double>(1,2), Tw.at<double>(1,0),
          Rw.at<double>(2,0), Rw.at<double>(2,1), Rw.at<double>(2,2), Tw.at<double>(2,0),
          0.0, 0.0, 0.0, 1.0;
    
    t = _t;
    viewer.addCoordinateSystem(2.0, t);


	

	viewer.addPointCloud(cloud,"keys");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"keys");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0,0,"keys");

	viewer.addPointCloud(cloud,"keys2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"keys2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1,0,"keys2");

	
	viewer.addSphere(final_pos,1,1,1,1,"final_pos");

	return final_pos;
}
*/
bool sort_func (avg_dist_t i,avg_dist_t j) { return (i.dist>j.dist); }

void re_calibrate_cams(vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &clouds, vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer) {


	vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clouds_sampled(clouds.size());
	vector<pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr> kdtrees(clouds.size());
	pcl::VoxelGrid<pcl::PointXYZINormal> sor;
		std::vector<int> nIdx;
		std::vector<float> nDist;
	
	for (int i = 0; i<  clouds.size();i++) {
		clouds_sampled[i] = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<PointXYZINormal>);
		sor.setInputCloud(clouds[i]);
		sor.setLeafSize (SAMPLE_SIZE, SAMPLE_SIZE, SAMPLE_SIZE);
		sor.filter (*clouds_sampled[i]);
		kdtrees[i] = pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
		kdtrees[i]->setInputCloud (clouds_sampled[i]);
	}


	vector<avg_dist_t> avg_dists;
	for (int i = 0; i < clouds_sampled.size(); i++) {
		float dist = 0;
		int cnt = 0;
		for (int k = 0; k < clouds_sampled.size(); k++) {
			if (k==i) continue;
			int local_cnt = 0;
			float local_dist = 0;
		
			for (int j = 0; j<  clouds_sampled[i]->points.size();j++) {
				
				
				kdtrees[k]->nearestKSearch(clouds_sampled[i]->points[j],1,nIdx,nDist);
				float normal_dist = 0; 
				normal_dist += fabs(clouds_sampled[i]->points[j].normal_x - clouds_sampled[k]->points[nIdx[0]].normal_x); 
				normal_dist += fabs(clouds_sampled[i]->points[j].normal_y - clouds_sampled[k]->points[nIdx[0]].normal_y); 
				normal_dist += fabs(clouds_sampled[i]->points[j].normal_z - clouds_sampled[k]->points[nIdx[0]].normal_z); 
				normal_dist /= 3.0;
				
				//if (local_dist < nDist[0]) local_dist = nDist[0];
				local_dist += /*nDist[0] +*/ normal_dist/**NORMAL_DIST_W*/;
				++local_cnt;
			}
			local_dist = local_dist / (float)local_cnt;
			//if (dist < local_dist) dist = local_dist;
			dist += local_dist;
			++cnt;
		}
		dist = dist / (float)cnt;
		cout << "cam: " << i << "dist: " << dist << endl;
		avg_dist_t aux;
		aux.index = i;
		aux.dist = dist;
		avg_dists.push_back(aux);
	}
	 
	std::sort(avg_dists.begin(),avg_dists.end(),sort_func);

	for (int i = 0; i < avg_dists.size(); i++) {
		cout << "Hola: " << avg_dists[i].index << " " << avg_dists[i].dist << endl;
	}

	cv::Mat       Rw, Tw;
	Eigen::Matrix4f   _t,_t2;
	Eigen::Affine3f   t,t2;
	Eigen::Affine3f final, min_final;
	cv::Mat rot_vec(3,1,CV_64F);
	cv::Mat rot_mat;
	 pcl::PointCloud<pcl::PointXYZINormal>::Ptr transf_cloud(new pcl::PointCloud<PointXYZINormal>);
	  pcl::PointCloud<pcl::PointXYZINormal>::Ptr doublesampled_cloud(new pcl::PointCloud<PointXYZINormal>);
	vector<float> min_dists;
	for (int idx = 0; idx < avg_dists.size(); idx++) {
		
		int i = avg_dists[idx].index;
		if (avg_dists[idx].dist < CALIB_THRESH) continue;
		cout << "Recalibrating cam " << i << ". Used cameras -> ";
		vector<int> confident_cams;
		for (int j = 0; j < clouds_sampled.size(); j++) {
			if (i==avg_dists[j].index) continue;
			//if (avg_dists[idx].dist > avg_dists[j].dist) {
			if (avg_dists[j].dist < CALIB_THRESH) {
				confident_cams.push_back(avg_dists[j].index);
				cout << avg_dists[j].index << " ";
			}
		}
		cout << endl;
		if (confident_cams.size()==0) continue;
		sor.setInputCloud (clouds[i]);
		sor.setLeafSize (SAMPLE_SIZE*2, SAMPLE_SIZE*2, SAMPLE_SIZE*2);
		sor.filter (*doublesampled_cloud);
		
		Rw =  params[i].rot.t();
		Tw = -Rw*params[i].trans;

		_t << Rw.at<double>(0,0), Rw.at<double>(0,1), Rw.at<double>(0,2), Tw.at<double>(0,0),
          Rw.at<double>(1,0), Rw.at<double>(1,1), Rw.at<double>(1,2), Tw.at<double>(1,0),
          Rw.at<double>(2,0), Rw.at<double>(2,1), Rw.at<double>(2,2), Tw.at<double>(2,0),
          0.0, 0.0, 0.0, 1.0;
    
		t = _t;
		float min_dist = avg_dists[idx].dist;
		cv::Mat min_rot_mat(3,3,CV_64F); 
		params[i].rot.copyTo(min_rot_mat);
		cv::Mat min_trans_vec(3,1,CV_64F);

		params[i].trans.copyTo(min_trans_vec);
		//cout << "Hola" << endl;
		for (float x_idx = -X_RANGE; x_idx <= X_RANGE; x_idx+= CALIB_STEP) {
			for (float y_idx = -Y_RANGE; y_idx <= Y_RANGE; y_idx+= CALIB_STEP) {
				for (float z_idx = -Z_RANGE; z_idx <= Z_RANGE; z_idx+= CALIB_STEP_R) {
					for (float rx_idx = -RX_RANGE; rx_idx <= RX_RANGE; rx_idx+= CALIB_STEP_R) {
						for (float ry_idx = -RY_RANGE; ry_idx <= RY_RANGE; ry_idx+= CALIB_STEP_R) {
							for (float rz_idx = -RZ_RANGE; rz_idx <= RZ_RANGE; rz_idx+= CALIB_STEP_R) {
						//		cout << "Hola dentro" << endl;
								rot_vec.at<double>(0) = rx_idx;
								rot_vec.at<double>(1) = ry_idx;
								rot_vec.at<double>(2) = rz_idx;
								cv::Rodrigues(rot_vec,rot_mat);
	 

								_t2 << rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2), x_idx,
								rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2), y_idx,
								rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2), z_idx,
								0.0, 0.0, 0.0, 1.0;
								t2 = _t2;
								//t = t*t2;
								_t2 = _t*_t2;
								//t = _t2;
								final = t*t2*t.inverse();
								//viewer.removeAllPointClouds();
								//pcl::transformPointCloud(*clouds[i],*transf_cloud,final);
								//viewer.addPointCloud(transf_cloud,"transformed");
								//cvWaitKey();
								pcl::transformPointCloudWithNormals(*doublesampled_cloud,*transf_cloud,final);
								
								
								float dist = 0;
								int cnt = 0;
								for (int k = 0; k < 1;/*confident_cams.size();*/ k++) {
									int local_cnt = 0;
									float local_dist = 0;
									for (int p = 0; p < transf_cloud->points.size();p++) {
										kdtrees[confident_cams[k]]->nearestKSearch(transf_cloud->points[p],1,nIdx,nDist);
										float normal_dist = 0; 
										normal_dist += fabs(transf_cloud->points[p].normal_x - clouds_sampled[confident_cams[k]]->points[nIdx[0]].normal_x); 
										normal_dist += fabs(transf_cloud->points[p].normal_y - clouds_sampled[confident_cams[k]]->points[nIdx[0]].normal_y); 
										normal_dist += fabs(transf_cloud->points[p].normal_z - clouds_sampled[confident_cams[k]]->points[nIdx[0]].normal_z); 
										normal_dist /= 3.0;
										local_dist += /*nDist[0] + */normal_dist/**NORMAL_DIST_W*/;
										++local_cnt;
									}
									local_dist = local_dist / (float)local_cnt;
									dist += local_dist;
									++cnt;
								}
								dist = dist / (float)cnt;
								if (dist < min_dist) {
									
									min_final = final;
									min_dist = dist;
									min_rot_mat.at<double>(0,0) = _t2(0,0);
									min_rot_mat.at<double>(0,1) = _t2(0,1);
									min_rot_mat.at<double>(0,2) = _t2(0,2);
									min_rot_mat.at<double>(1,0) = _t2(1,0);
									min_rot_mat.at<double>(1,1) = _t2(1,1);
									min_rot_mat.at<double>(1,2) = _t2(1,2);
									min_rot_mat.at<double>(2,0) = _t2(2,0);
									min_rot_mat.at<double>(2,1) = _t2(2,1);
									min_rot_mat.at<double>(2,2) = _t2(2,2);
									min_rot_mat = min_rot_mat.t();

									min_trans_vec.at<double>(0) = _t2(0,3);
									min_trans_vec.at<double>(1) = _t2(1,3);
									min_trans_vec.at<double>(2) = _t2(2,3);

									min_trans_vec = -min_rot_mat*min_trans_vec;
								}
								//cout << "cam: " << i << "dist: " << dist << endl;
								//avg_dists.push_back(dist);
								//viewer.addCoordinateSystem(2.0, t*t2);
							}
						}
					}
				}
			}
		}
		pcl::transformPointCloudWithNormals(*clouds[i],*clouds[i],min_final);
		sor.setInputCloud(clouds[i]);
		sor.setLeafSize (SAMPLE_SIZE, SAMPLE_SIZE, SAMPLE_SIZE);
		sor.filter (*clouds_sampled[i]);
		kdtrees[i]->setInputCloud (clouds_sampled[i]);
		min_dists.push_back(min_dist);
		//cout << "ROT: " << endl;
		//cout << params[i].rot << endl;
		//cout << min_rot_mat << endl;
		//cout << "TRANS: " << endl;
		//cout << params[i].trans << endl;
		//cout << min_trans_vec << endl;
		
		params[i].rot = min_rot_mat;
		params[i].trans = min_trans_vec;
		cout << "cam: " << i << "dist: " << min_dist << endl;
	}


	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr all_clouds(new pcl::PointCloud<PointXYZ>);
	for (int i = 0; i<  clouds.size();i++) {
		*all_clouds += *clouds[i];
		
	}
	viewer.addPointCloud(all_clouds,"all_clouds");
	*/
}

void prueba(vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clouds,pcl::visualization::PCLVisualizer &viewer) {

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr all_clouds(new PointCloud<PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr all_clouds_sampled(new PointCloud<PointXYZINormal>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new PointCloud<pcl::Normal>);
	pcl::VoxelGrid<pcl::PointXYZINormal> sor;
	/*for (int i = 0; i < clouds.size(); i++) {
			*all_clouds += *clouds[i];
	}
	*/
	all_clouds = clouds[7];
	
	
	sor.setInputCloud(all_clouds);
	sor.setLeafSize (SAMPLE_SIZE*2, SAMPLE_SIZE*2, SAMPLE_SIZE*2);
	sor.filter (*all_clouds_sampled);

	cv::Mat rot_vec(3,1,CV_64F);
	cv::Mat rot_mat(3,3,CV_64F);
	rot_vec.at<double>(0) = CV_PI/4;
	rot_vec.at<double>(1) = 0;
	rot_vec.at<double>(2) = 0;
	cv::Rodrigues(rot_vec,rot_mat);
	 Eigen::Matrix4f   _t2;
	Eigen::Affine3f   t2;

	_t2 << rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2), 0,
	rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2), 0,
	rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2), 0,
	0.0, 0.0, 0.0, 1.0;
	t2 = _t2;
	
	pcl::transformPointCloudWithNormals(*all_clouds_sampled,*all_clouds_sampled,t2);
	
	for (int i = 0; i < all_clouds_sampled->points.size();i++) {
		pcl::Normal n;
		n.normal_x = all_clouds_sampled->points[i].normal_x;
		n.normal_y = all_clouds_sampled->points[i].normal_y;
		n.normal_z = all_clouds_sampled->points[i].normal_z;
		normal_cloud->points.push_back(n);
	}

	viewer.addPointCloudNormals<PointXYZINormal,Normal>(all_clouds_sampled,normal_cloud,1,10,"normals_clouds");
	
}

 vector<vector<vector<vector<matchings_t>>>> refine_depth_constraints(vector<vector<vector<cv::Mat>>> &normal_maps,vector<vector<vector<PointXYZINormal>>> surfaces, vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer) {


	 int im_height = surfaces[0].size();
	 int im_width = surfaces[0][0].size();
	vector<vector<vector<vector<matchings_t>>>> matches3D(CAM_NUM,vector<vector<vector<matchings_t>>>(im_height,vector<vector<matchings_t>>(im_width)));
	
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr all_clouds(new pcl::PointCloud<PointXYZINormal>);
	pcl::PointCloud<pcl::Normal>::Ptr all_normals(new pcl::PointCloud<Normal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr sampled_all_clouds(new pcl::PointCloud<PointXYZINormal>);
	
	vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clouds(CAM_NUM);
	vector<pcl::PointCloud<pcl::Normal>::Ptr> normals(CAM_NUM);
	vector<vector<cv::Point>> clouds_pixel_idx(CAM_NUM);
	for (int i = 0; i< surfaces.size(); i++) {
		clouds[i] = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<PointXYZINormal>);
		normals[i] = pcl::PointCloud<Normal>::Ptr(new pcl::PointCloud<Normal>);
		for (int u = 0; u< surfaces[i].size(); u+=2) {
			for (int v = 0; v< surfaces[i][0].size(); v+=2) {
				if (surfaces[i][u][v].x == -1000) continue;
				clouds[i]->points.push_back(surfaces[i][u][v]);
				cv::Point p;
				p.x = v;
				p.y = u;
				clouds_pixel_idx[i].push_back(p);
				//normals[i]->points.push_back(pcl::Normal(surfaces[i][u][v].normal_x,surfaces[i][u][v].normal_y,surfaces[i][u][v].normal_z));
			}
		}
	}
	
	/*vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> aux_clouds(CAM_NUM);
	for (int i = 0; i< CAM_NUM; i++) {
		aux_clouds[i] = pcl::PointCloud<PointXYZ>::Ptr(new pcl::PointCloud<PointXYZ>);
		for (int j = 0; j < clouds[i]->points.size();j++) { 
			PointXYZ p;
			p.x = clouds[i]->points[j].x;
			p.y = clouds[i]->points[j].y;
			p.z = clouds[i]->points[j].z;
			aux_clouds[i]->points.push_back(p);
		}
	}
	*/
	pcl::NormalEstimation<PointXYZINormal, pcl::Normal> ne;
	
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal> ());
	ne.setSearchMethod (tree);
	
	for (int i = 0; i < CAM_NUM; i++) {
		ne.setInputCloud (clouds[i]);
		ne.setKSearch(20);
		ne.compute (*normals[i]);
		for (int j = 0; j < normals[i]->points.size(); j++) {
			pcl::flipNormalTowardsViewpoint<PointXYZINormal>(clouds[i]->points[j],params[i].pos.at<double>(0),
															params[i].pos.at<double>(1),params[i].pos.at<double>(2),
															normals[i]->points[j].normal_x,normals[i]->points[j].normal_y,normals[i]->points[j].normal_z);
		}

	}
	
	for (int i = 0; i < CAM_NUM;i++) {
		*all_clouds = *all_clouds + *clouds[i];
		*all_normals = *all_normals + *normals[i];
	}
	/*
	pcl::VoxelGrid<pcl::PointXYZINormal> sor;
	

	sor.setInputCloud (all_clouds);
	sor.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
	sor.filter (*sampled_all_clouds);
	*/

	sampled_all_clouds = all_clouds;
	/*for (int i = 0; i < CAM_NUM;i++) {
		sor.setInputCloud (clouds[i]);
		sor.setLeafSize (1, 1, 1);
		sor.filter (*clouds[i]);

	}
	*/

	pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtree_ptr (new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
	kdtree_ptr->setInputCloud (sampled_all_clouds);
	std::vector<int> nIdx;
	std::vector<float> nDist;
	Eigen::Vector4f line_point, line_dir,ext_point;
	line_point[3] = 0;
	line_dir[3] = 0;
	ext_point[3] = 0;
	for (int i = 0; i < CAM_NUM;i++) {

		cv::Mat M = roto_translation(params[i].rot,params[i].trans);
		cout << "Refining depth constraints for cam " << i << endl;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr avg_cloud(new pcl::PointCloud<PointXYZINormal>);
		
		/*
		viewer.removeAllPointClouds();
		//viewer.removeShape("sphere");
		//viewer.addSphere(PointXYZ(clouds[i]->points[0].x,clouds[i]->points[0].y,clouds[i]->points[0].z),4,1,0,0,"sphere");
		viewer.addPointCloud<PointXYZINormal>(sampled_all_clouds,"all_clouds");
		*/
		for (int j = 0; j < clouds[i]->points.size();j+=10) { 

			cv::Mat nw(4,1,CV_64F);
			nw.at<double>(0) = normals[i]->points[j].normal_x;
			nw.at<double>(1) = normals[i]->points[j].normal_y;
			nw.at<double>(2) = normals[i]->points[j].normal_z;
			nw.at<double>(3) = 1.0;
			cv::Mat n = M*nw;

			normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x].at<double>(0) = n.at<double>(0)-params[i].trans.at<double>(0);
			normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x].at<double>(1) = n.at<double>(1)-params[i].trans.at<double>(1);
			normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x].at<double>(2) = n.at<double>(2)-params[i].trans.at<double>(2);
			normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x] = normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x]/calc_norm(normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x],3);
			

			line_dir[0] = params[i].pos.at<double>(0) - clouds[i]->points[j].x;//normals[i]->points[j].normal_x;//clouds[i]->points[j].normal_x;
			line_dir[1] = params[i].pos.at<double>(1) - clouds[i]->points[j].y;//normals[i]->points[j].normal_y;//clouds[i]->points[j].normal_y;
			line_dir[2] = params[i].pos.at<double>(2) - clouds[i]->points[j].z;//normals[i]->points[j].normal_z;//clouds[i]->points[j].normal_z;
			line_point[0] = clouds[i]->points[j].x;
			line_point[1] = clouds[i]->points[j].y;
			line_point[2] = clouds[i]->points[j].z;

			normals[i]->points[j].normal_x = line_dir[0];
			normals[i]->points[j].normal_y = line_dir[1];
			normals[i]->points[j].normal_z = line_dir[2];

			Eigen::Vector4f dir1,dir2;
			dir1[3] = 0;
			dir2[3] = 0;
			dir1[0] = normals[i]->points[j].normal_x;
			dir1[1] = normals[i]->points[j].normal_y;
			dir1[2] = normals[i]->points[j].normal_z;
			dir2[0] = line_dir[0];
			dir2[1] = line_dir[1];
			dir2[2] = line_dir[2];


			
			float ang = pcl::getAngle3D(dir1,dir2);
			if (ang > CV_PI/2.5) continue;
			kdtree_ptr->radiusSearch(clouds[i]->points[j],REFIN_SEARCH_R,nIdx,nDist);
		//	cout<<i<<" "<<j<<" "<<nIdx.size()<<endl;
			PointXYZINormal p;
			p.x = 0;
			p.y = 0;
			p.z = 0;
			int cnt = 0;
			for (int k = 1; k < nIdx.size();k++) {
				ext_point[0] = sampled_all_clouds->points[nIdx[k]].x;
				ext_point[1] = sampled_all_clouds->points[nIdx[k]].y;
				ext_point[2] = sampled_all_clouds->points[nIdx[k]].z;
				double dist = std::sqrt(pcl::sqrPointToLineDistance(ext_point,line_point,line_dir));
				dir1[0] = all_normals->points[nIdx[k]].normal_x;
				dir1[1] = all_normals->points[nIdx[k]].normal_y;
				dir1[2] = all_normals->points[nIdx[k]].normal_z;
				float ang2 = pcl::getAngle3D(dir1,dir2);
				if ((dist < DIRECTED_NEIGH_RADIAL_DIST) && (ang2 < CV_PI/2.5)) {
					p.x += sampled_all_clouds->points[nIdx[k]].x;
					p.y += sampled_all_clouds->points[nIdx[k]].y;
					p.z += sampled_all_clouds->points[nIdx[k]].z;
					++cnt;
				}
				
			}
			if (cnt > 0) {
				p.x /= (float)cnt;
				p.y /= (float)cnt;
				p.z /= (float)cnt;
				//cout << p<< endl;
				avg_cloud->points.push_back(p);
				matchings_t aux;
				aux.point.x = p.x;
				aux.point.y = p.y;
				aux.point.z = p.z;


				matches3D[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x].push_back(aux);
			}
		}
		/*
		viewer.addPointCloud<PointXYZINormal>(avg_cloud,"avg_cloud");
		viewer.addPointCloudNormals<PointXYZINormal,Normal>(clouds[i],normals[i],1,1,"orig_cloud");
		//viewer.addPointCloud<PointXYZINormal>(clouds[i],"orig_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"orig_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0,"orig_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"avg_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1,"avg_cloud");
		cvWaitKey();
		*/
		//cout << "Hola" << endl;
	
	}
	return matches3D;
}


vector<vector<vector<vector<matchings_t>>>> refine_depth_constraints_one_cam_old(int cam_idx, vector<vector<vector<cv::Mat>>> &normal_maps,vector<vector<vector<PointXYZINormal>>> surfaces, vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer) {


	 int im_height = surfaces[0].size();
	 int im_width = surfaces[0][0].size();
	 float fscale = float(normal_maps[cam_idx][0].size())/640.0;
	vector<vector<vector<vector<matchings_t>>>> matches3D(CAM_NUM,vector<vector<vector<matchings_t>>>(im_height,vector<vector<matchings_t>>(im_width)));
	
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr all_clouds(new pcl::PointCloud<PointXYZINormal>);
	pcl::PointCloud<pcl::Normal>::Ptr all_normals(new pcl::PointCloud<Normal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr sampled_all_clouds(new pcl::PointCloud<PointXYZINormal>);
	
	vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clouds(CAM_NUM);
	vector<pcl::PointCloud<pcl::Normal>::Ptr> normals(CAM_NUM);
	vector<vector<cv::Point>> clouds_pixel_idx(CAM_NUM);
	for (int i = 0; i< surfaces.size(); i++) {
		clouds[i] = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<PointXYZINormal>);
		normals[i] = pcl::PointCloud<Normal>::Ptr(new pcl::PointCloud<Normal>);
		for (int u = 0; u< surfaces[i].size(); u+=2) {
			for (int v = 0; v< surfaces[i][0].size(); v+=2) {
				if (surfaces[i][u][v].x == -1000) continue;
				clouds[i]->points.push_back(surfaces[i][u][v]);
				cv::Point p;
				p.x = v;
				p.y = u;
				clouds_pixel_idx[i].push_back(p);
				//normals[i]->points.push_back(pcl::Normal(surfaces[i][u][v].normal_x,surfaces[i][u][v].normal_y,surfaces[i][u][v].normal_z));
			}
		}
	}
	
	/*vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> aux_clouds(CAM_NUM);
	for (int i = 0; i< CAM_NUM; i++) {
		aux_clouds[i] = pcl::PointCloud<PointXYZ>::Ptr(new pcl::PointCloud<PointXYZ>);
		for (int j = 0; j < clouds[i]->points.size();j++) { 
			PointXYZ p;
			p.x = clouds[i]->points[j].x;
			p.y = clouds[i]->points[j].y;
			p.z = clouds[i]->points[j].z;
			aux_clouds[i]->points.push_back(p);
		}
	}
	*/
	pcl::NormalEstimation<PointXYZINormal, pcl::Normal> ne;
	
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal> ());
	ne.setSearchMethod (tree);
	
	for (int i = 0; i < CAM_NUM; i++) {
		ne.setInputCloud (clouds[i]);
		ne.setKSearch(20);
		ne.compute (*normals[i]);
		for (int j = 0; j < normals[i]->points.size(); j++) {
			pcl::flipNormalTowardsViewpoint<PointXYZINormal>(clouds[i]->points[j],params[i].pos.at<double>(0),
															params[i].pos.at<double>(1),params[i].pos.at<double>(2),
															normals[i]->points[j].normal_x,normals[i]->points[j].normal_y,normals[i]->points[j].normal_z);
		}

	}
	
	for (int i = 0; i < CAM_NUM;i++) {
		*all_clouds = *all_clouds + *clouds[i];
		*all_normals = *all_normals + *normals[i];
	}
	/*
	pcl::VoxelGrid<pcl::PointXYZINormal> sor;
	

	sor.setInputCloud (all_clouds);
	sor.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
	sor.filter (*sampled_all_clouds);
	*/

	sampled_all_clouds = all_clouds;
	/*for (int i = 0; i < CAM_NUM;i++) {
		sor.setInputCloud (clouds[i]);
		sor.setLeafSize (1, 1, 1);
		sor.filter (*clouds[i]);

	}
	*/

	pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtree_ptr (new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
	kdtree_ptr->setInputCloud (sampled_all_clouds);
	std::vector<int> nIdx;
	std::vector<float> nDist;
	Eigen::Vector4f line_point, line_dir,ext_point;
	line_point[3] = 0;
	line_dir[3] = 0;
	ext_point[3] = 0;
	for (int i = cam_idx; i <= cam_idx;i++) {

		cv::Mat M = roto_translation(params[i].rot,params[i].trans);
		cout << "Refining depth constraints for cam " << i << endl;
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr avg_cloud(new pcl::PointCloud<PointXYZINormal>);
		
		/*
		viewer.removeAllPointClouds();
		//viewer.removeShape("sphere");
		//viewer.addSphere(PointXYZ(clouds[i]->points[0].x,clouds[i]->points[0].y,clouds[i]->points[0].z),4,1,0,0,"sphere");
		viewer.addPointCloud<PointXYZINormal>(sampled_all_clouds,"all_clouds");
		*/
		for (int j = 0; j < clouds[i]->points.size();j+=2) { 

			cv::Mat nw(4,1,CV_64F);
			nw.at<double>(0) = normals[i]->points[j].normal_x;
			nw.at<double>(1) = normals[i]->points[j].normal_y;
			nw.at<double>(2) = normals[i]->points[j].normal_z;
			nw.at<double>(3) = 1.0;
			cv::Mat n = M*nw;

			normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x].at<double>(0) = n.at<double>(0)-params[i].trans.at<double>(0);
			normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x].at<double>(1) = n.at<double>(1)-params[i].trans.at<double>(1);
			normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x].at<double>(2) = n.at<double>(2)-params[i].trans.at<double>(2);
			normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x] = normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x]/calc_norm(normal_maps[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x],3);
			

			line_dir[0] = params[i].pos.at<double>(0) - clouds[i]->points[j].x;//normals[i]->points[j].normal_x;//clouds[i]->points[j].normal_x;
			line_dir[1] = params[i].pos.at<double>(1) - clouds[i]->points[j].y;//normals[i]->points[j].normal_y;//clouds[i]->points[j].normal_y;
			line_dir[2] = params[i].pos.at<double>(2) - clouds[i]->points[j].z;//normals[i]->points[j].normal_z;//clouds[i]->points[j].normal_z;
			line_point[0] = clouds[i]->points[j].x;
			line_point[1] = clouds[i]->points[j].y;
			line_point[2] = clouds[i]->points[j].z;

			normals[i]->points[j].normal_x = line_dir[0];
			normals[i]->points[j].normal_y = line_dir[1];
			normals[i]->points[j].normal_z = line_dir[2];

			Eigen::Vector4f dir1,dir2;
			dir1[3] = 0;
			dir2[3] = 0;
			dir1[0] = normals[i]->points[j].normal_x;
			dir1[1] = normals[i]->points[j].normal_y;
			dir1[2] = normals[i]->points[j].normal_z;
			dir2[0] = line_dir[0];
			dir2[1] = line_dir[1];
			dir2[2] = line_dir[2];


			
			float ang = pcl::getAngle3D(dir1,dir2);
			//if (ang > CV_PI/2.5) continue;
			kdtree_ptr->radiusSearch(clouds[i]->points[j],REFIN_SEARCH_R,nIdx,nDist);
		//	cout<<i<<" "<<j<<" "<<nIdx.size()<<endl;
			PointXYZINormal p;
			p.x = 0;
			p.y = 0;
			p.z = 0;
			int cnt = 0;
			for (int k = 1; k < nIdx.size();k++) {
				ext_point[0] = sampled_all_clouds->points[nIdx[k]].x;
				ext_point[1] = sampled_all_clouds->points[nIdx[k]].y;
				ext_point[2] = sampled_all_clouds->points[nIdx[k]].z;
				double dist = std::sqrt(pcl::sqrPointToLineDistance(ext_point,line_point,line_dir));
				dir1[0] = all_normals->points[nIdx[k]].normal_x;
				dir1[1] = all_normals->points[nIdx[k]].normal_y;
				dir1[2] = all_normals->points[nIdx[k]].normal_z;
				float ang2 = pcl::getAngle3D(dir1,dir2);
				if ((dist < DIRECTED_NEIGH_RADIAL_DIST) && (ang2 < CV_PI/2.5)) { //2.5
					p.x += sampled_all_clouds->points[nIdx[k]].x;
					p.y += sampled_all_clouds->points[nIdx[k]].y;
					p.z += sampled_all_clouds->points[nIdx[k]].z;
					++cnt;
				}
				
			}
			if (cnt > 0) {
				p.x /= (float)cnt;
				p.y /= (float)cnt;
				p.z /= (float)cnt;
				//cout << p<< endl;
				avg_cloud->points.push_back(p);
				matchings_t aux;
				aux.point.x = p.x;
				aux.point.y = p.y;
				aux.point.z = p.z;

				cv::Mat Pl = compound_cam_transformation(params[i].cam,params[i].rot,params[i].trans);
				cv::Mat p3d(4,1,CV_64F);
				p3d.at<double>(0) = p.x;
				p3d.at<double>(1) = p.y;
				p3d.at<double>(2) = p.z;
				p3d.at<double>(3) = 1.0;

				cv::Mat p2d = Pl*p3d;
				p2d.at<double>(0) /= p2d.at<double>(2);
				p2d.at<double>(1) /= p2d.at<double>(2);


				//cout << "X: " << p2d.at<double>(0)*fscale << " Y: " << p2d.at<double>(1)*fscale << endl;
				//matches3D[i][clouds_pixel_idx[i][j].y][clouds_pixel_idx[i][j].x].push_back(aux);
				matches3D[i][p2d.at<double>(1)*fscale][p2d.at<double>(0)*fscale].push_back(aux);
			}
		}
		/*
		viewer.addPointCloud<PointXYZINormal>(avg_cloud,"avg_cloud");
		viewer.addPointCloudNormals<PointXYZINormal,Normal>(clouds[i],normals[i],1,1,"orig_cloud");
		//viewer.addPointCloud<PointXYZINormal>(clouds[i],"orig_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"orig_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0,"orig_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"avg_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1,"avg_cloud");
		cvWaitKey();
		*/
		//cout << "Hola" << endl;
	
	}
	return matches3D;
}

bool rayTriangleIntersection(cv::Mat p, cv::Mat d, cv::Mat v0, cv::Mat v1, cv::Mat v2, double &t) {

	cv::Mat e1(3,1,CV_64F);
	cv::Mat e2(3,1,CV_64F);
	cv::Mat h(3,1,CV_64F);
	cv::Mat s(3,1,CV_64F);
	cv::Mat q(3,1,CV_64F);
	double a,f,u,v;

	e1 = v1-v0;
	e2 = v2-v0;
	h = d.cross(e2);
	a = e1.dot(h);
	if (a > -0.00001 && a < 0.00001)
		return false;
	f = 1/a;
	s = p-v0;
	u = f*s.dot(h);

	if (u < 0.0 || u > 1.0) {
		return false;
	}
	q = s.cross(e1);
	v = f * d.dot(q);
	if (v < 0.0 || u + v > 1.0)
		return(false);
	t = f * e2.dot(q);
	if (t > 0.00001) // ray intersection
		return(true);
	else // this means that there is a line intersection
		 // but not a ray intersection
		 return (false);
	
}

void filter_surface_proxy_hull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<vector<vector<PointXYZINormal>>> surfaces, vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer, pcl::PolygonMesh triangles, float ray_step) {
	
	vector<vector<vector<vector<matchings_t>>>> intersections = proxy_hull_ray_intersections(cloud,surfaces,params,viewer,triangles, 1);
	for (int i = 0; i < intersections.size(); i++) {
		for (int j = 0; j < intersections[0].size(); j++) {
			for (int k = 0; k < intersections[0][0].size(); k++) {
				if (intersections[i][j][k].size() == 0) {
					surfaces[i][j][k].x = -1000;
					surfaces[i][j][k].y = -1000;
					surfaces[i][j][k].z = -1000;
				}
			}	
		}
	}
}

vector<vector<vector<vector<matchings_t>>>> proxy_hull_ray_intersections(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<vector<vector<PointXYZINormal>>> surfaces, vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer, pcl::PolygonMesh triangles, float ray_step) {


	int im_height = surfaces[0].size();
	int im_width = surfaces[0][0].size();
	vector<vector<vector<vector<matchings_t>>>> matches3D(CAM_NUM,vector<vector<vector<matchings_t>>>(im_height,vector<vector<matchings_t>>(im_width)));

	float scalef = (double)surfaces[0][0].size()/640.0;

	cv::Mat v0(3,1,CV_64F);
	cv::Mat v1(3,1,CV_64F);
	cv::Mat v2(3,1,CV_64F);
	double t;
	
	
	for (int i = 0; i < CAM_NUM; i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr intersections_cloud(new pcl::PointCloud<PointXYZ>);
		cv::Mat Pl = compound_cam_transformation(params[i].cam,params[i].rot,params[i].trans);
		for (int u = 0; u< surfaces[i].size(); u+=ray_step) {
			for (int v = 0; v< surfaces[i][0].size(); v+=ray_step) {
				if (surfaces[i][u][v].x == -1000) continue;
				cv::Mat p1(3,1,CV_64F);
				p1.at<double>(0) = (double)v/scalef;
				p1.at<double>(1) = (double)u/scalef;
				p1.at<double>(2) = 1.0;
				cv::Mat aux = Pl.t()*(Pl*Pl.t()).inv()*p1;

				aux /= aux.at<double>(3);
				cv::Mat l(3,1,CV_64F);
		
				l.at<double>(0) = aux.at<double>(0) - params[i].pos.at<double>(0);
				l.at<double>(1) = aux.at<double>(1) - params[i].pos.at<double>(1);
				l.at<double>(2) = aux.at<double>(2) - params[i].pos.at<double>(2);

				l = l / calc_norm(l,3);

				matchings_t best_match;
				float min_dist = 99999;
				for (int k = 0; k < triangles.polygons.size() ; k++) {
					v0.at<double>(0) = cloud->points[triangles.polygons[k].vertices[0]].x;
					v0.at<double>(1) = cloud->points[triangles.polygons[k].vertices[0]].y;
					v0.at<double>(2) = cloud->points[triangles.polygons[k].vertices[0]].z;

					v1.at<double>(0) = cloud->points[triangles.polygons[k].vertices[1]].x;
					v1.at<double>(1) = cloud->points[triangles.polygons[k].vertices[1]].y;
					v1.at<double>(2) = cloud->points[triangles.polygons[k].vertices[1]].z;

					v2.at<double>(0) = cloud->points[triangles.polygons[k].vertices[2]].x;
					v2.at<double>(1) = cloud->points[triangles.polygons[k].vertices[2]].y;
					v2.at<double>(2) = cloud->points[triangles.polygons[k].vertices[2]].z;

					if (rayTriangleIntersection(params[i].pos, l, v0,v1, v2, t)) {
						cv::Mat p = params[i].pos+t*l;
						matchings_t match;
						match.point = PointXYZ(p.at<double>(0),p.at<double>(1),p.at<double>(2));
						
							PointXYZ cam_orig,new_p;
							new_p = match.point;
							
							cam_orig.x = params[i].pos.at<double>(0);
							cam_orig.y = params[i].pos.at<double>(1);
							cam_orig.z = params[i].pos.at<double>(2);
							
							
							float new_dist = sqrt(double((cam_orig.x-new_p.x)*(cam_orig.x-new_p.x)+(cam_orig.y-new_p.y)*(cam_orig.y-new_p.y)+(cam_orig.z-new_p.z)*(cam_orig.z-new_p.z)));
							
							if (new_dist < min_dist) {
								best_match = match;
								min_dist = new_dist;
							}
							//	matches3D[i][u][v][0] = match;
						
						//else {
						//	matches3D[i][u][v].push_back(match);
						//}
						//intersections_cloud->points.push_back(PointXYZ(p.at<double>(0),p.at<double>(1),p.at<double>(2)));
					}
				}
				if (min_dist < 99999) {
					matches3D[i][u][v].push_back(best_match);
				//	intersections_cloud->points.push_back(best_match.point);
				}
			}
		}
		//viewer.removePointCloud("intersections");
		//viewer.addPointCloud(intersections_cloud, "intersections");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)cam_col[i][0]/255.0, (float)cam_col[i][1]/255.0, (float)cam_col[i][2]/255.0,"intersections");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"intersections");
		//cvWaitKey();
	}
	
	return matches3D;
}



void create_avg_surface(vector<pcl::PointCloud<PointXYZINormal>::Ptr> clouds,pcl::visualization::PCLVisualizer &viewer, pcl::PolygonMesh &triangles,pcl::PointCloud<pcl::PointXYZ>::Ptr &mls_points, float voxel_size, float mu,float tri_max_r,float tri_max_ang,float tri_min_ang, float mls_radius) {
	
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr all_clouds(new pcl::PointCloud<PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr sampled_all_clouds(new pcl::PointCloud<PointXYZINormal>);
	for (int i = 0; i < clouds.size(); i++) {
		*all_clouds += *clouds[i];
	}

	pcl::VoxelGrid<pcl::PointXYZINormal> sor;
	

	sor.setInputCloud (all_clouds);
	sor.setLeafSize (voxel_size, voxel_size, voxel_size);
	cout << "voxel size: "<< voxel_size << endl;
	sor.filter (*sampled_all_clouds);

	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_all_clouds(new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud(new pcl::PointCloud<PointXYZ>);
	for (int i = 0; i < sampled_all_clouds->points.size(); i++) {
		PointXYZ p;
		p.x = sampled_all_clouds->points[i].x;
		p.y = sampled_all_clouds->points[i].y;
		p.z = sampled_all_clouds->points[i].z;
		smooth_all_clouds->points.push_back(p);
	}
	
	
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
 
	 mls.setComputeNormals (false);
	// Create a KD-Tree
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree (new pcl::search::KdTree<pcl::PointXYZ>);
	// Set parameters
	mls.setInputCloud (smooth_all_clouds);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (mls_tree);
	mls.setSearchRadius (mls_radius);

	// Reconstruct
	mls.process (*mls_points);
	
	
	//mls_points = smooth_all_clouds;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::ConvexHull<pcl::PointXYZ> chull;
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud (mls_points);
	chull.setAlpha(20);
	chull.reconstruct (*cloud_hull);
	
	chull.reconstruct(triangles);
	
	
	mls_points = cloud_hull;
	/*
	// Create a KD-Tree
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree (new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	 
	  
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
 
	 mls.setComputeNormals (false);

	// Set parameters
	mls.setInputCloud (cloud_hull);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (mls_tree);
	mls.setSearchRadius (mls_radius);

	// Reconstruct
	mls.process (*mls_points);
	
	  //mls_points = cloud_hull;

Binary file ./ps/bundleAdjustment.cpp matches
