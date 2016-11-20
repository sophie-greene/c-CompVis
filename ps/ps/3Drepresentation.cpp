#include "3Drepresentation.h"


#define CAM_AXIS_SIZE 3



using namespace std;
//using namespace cv;
using namespace pcl;


int cam_colors[CAM_NUM][3] = {{255,0,0},{0,255,0},{0, 0, 255},{255, 255, 0},{255, 0 ,255},{0, 255, 255},{255, 255, 255},{100, 100, 100}};


void add_cam_link(cam_params_t params1, cam_params_t params2,string name,pcl::visualization::PCLVisualizer &viewer) {
	
	PointXYZ p1,p2;
	cv::Mat pos = -params1.rot.t()*params1.trans;
	p1.x = pos.at<double>(0);
	p1.y = pos.at<double>(1);
	p1.z = pos.at<double>(2);

	pos = -params2.rot.t()*params2.trans;
	p2.x = pos.at<double>(0);
	p2.y = pos.at<double>(1);
	p2.z = pos.at<double>(2);

	viewer.addLine(p1,p2,name);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2,name);
}

void add_cameras_to_world(vector<cam_params_t> params,  pcl::visualization::PCLVisualizer &viewer,string name) {
	
	pcl::PointXYZ base,top;
	cv::Mat       Rw, Tw;
	Eigen::Matrix4f   _t;
    Eigen::Affine3f   t;
	for (int i = 0; i < params.size(); i++) {
		Rw =  params[i].rot.t();
		Tw = -Rw*params[i].trans;

		_t << Rw.at<double>(0,0), Rw.at<double>(0,1), Rw.at<double>(0,2), Tw.at<double>(0,0),
          Rw.at<double>(1,0), Rw.at<double>(1,1), Rw.at<double>(1,2), Tw.at<double>(1,0),
          Rw.at<double>(2,0), Rw.at<double>(2,1), Rw.at<double>(2,2), Tw.at<double>(2,0),
          0.0, 0.0, 0.0, 1.0;
    
		 t = _t;
		viewer.addCoordinateSystem(2.0, t);




		stringstream ss_label;
		ss_label << i;

		cv::Mat pos = -params[i].rot.t()*params[i].trans;
		base.x = pos.at<double>(0); 
		base.y = pos.at<double>(1); 
		base.z = pos.at<double>(2); 
		
		viewer.addText3D(ss_label.str(),base,1,(float)cam_colors[i][0]/255.0,(float)cam_colors[i][1]/255.0,(float)cam_colors[i][2]/255.0,name+"cam_no"+ss_label.str());
	/*	
		Mat M = roto_translation(params[i].rot,params[i].trans);
		
		Mat x_axis(4,1,CV_64F);
		Mat y_axis(4,1,CV_64F);
		Mat z_axis(4,1,CV_64F);

		x_axis.at<double>(0) = 1.0*CAM_AXIS_SIZE;
		x_axis.at<double>(1) = 0.0;
		x_axis.at<double>(2) = 0.0;
		x_axis.at<double>(3) = 1.0;

		y_axis.at<double>(0) = 0.0;
		y_axis.at<double>(1) = 1.0*CAM_AXIS_SIZE;
		y_axis.at<double>(2) = 0.0;
		y_axis.at<double>(3) = 1.0;

		z_axis.at<double>(0) = 0.0;
		z_axis.at<double>(1) = 0.0;
		z_axis.at<double>(2) = 1.0*CAM_AXIS_SIZE;
		z_axis.at<double>(3) = 1.0;

		x_axis = M.inv()*x_axis;
		y_axis = M.inv()*y_axis;
		z_axis = M.inv()*z_axis;

		//x_axis -= params[i].pos;
		//y_axis -= params[i].pos;
		//z_axis -= params[i].pos;

		//x_axis /= calc_norm(x_axis,3);
		//y_axis /= calc_norm(y_axis,3);
		//z_axis /= calc_norm(z_axis,3);

		top.x = x_axis.at<double>(0);
		top.y = x_axis.at<double>(1);
		top.z = x_axis.at<double>(2);
		
		stringstream ss;
		ss << i;
		string axis_name = name + "x_axis" + ss.str();
		viewer.addLine(base,top,axis_name);
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,1,axis_name);
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,1,axis_name);

		top.x = y_axis.at<double>(0);
		top.y = y_axis.at<double>(1);
		top.z = y_axis.at<double>(2);
		
		axis_name = name + "y_axis" + ss.str();
		viewer.addLine(base,top,axis_name);
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,1,axis_name);
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,axis_name);

		top.x = z_axis.at<double>(0);
		top.y = z_axis.at<double>(1);
		top.z = z_axis.at<double>(2);
		
		axis_name =name +  "z_axis" + ss.str();
		viewer.addLine(base,top,axis_name);
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,1,axis_name);
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,axis_name);
		*/
	}
	
	add_cam_link(params[0],params[5],name+"link05",viewer);
	add_cam_link(params[5],params[4],name+"link54",viewer);
	add_cam_link(params[3],params[4],name+"link34",viewer);
	add_cam_link(params[6],params[1],name+"link61",viewer);
	add_cam_link(params[6],params[7],name+"link67",viewer);
	add_cam_link(params[7],params[2],name+"link72",viewer);
	
}

void add_matchings_to_cloud(IplImage* im,pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<vector<cv::Point> > points, cam_params_t params1, cam_params_t params2,int &cnt) {


	
	float color_r = (rand()%100+155)/255.0;
	float color_g = (rand()%100+155)/255.0;
	float color_b = (rand()%100+155)/255.0;

	cv::Mat Pl = compound_cam_transformation(params1.cam,params1.rot,params1.trans);
		cv::Mat Pr = compound_cam_transformation(params2.cam,params2.rot,params2.trans);
		
		//cout << "Pl: " << Pl << endl;
		//cout << "Pr: " << Pr << endl;

		

	for (int i = 0; i < points[0].size(); i++) {

		//cout << points[0][i] << endl
		//	 << points[1][i] << endl;

		cvCircle(im,points[0][i],2,cvScalar(0,255,255),2);

		++cnt;
		cv::Mat p1(3,1,CV_64F);
		cv::Mat p2(3,1,CV_64F);
		
		p1.at<double>(0) = (double)(points[0][i].x);
		p1.at<double>(1) = (double)(points[0][i].y);
		p1.at<double>(2) = 1.0;

		p2.at<double>(0) = (double)(points[1][i].x);
		p2.at<double>(1) = (double)(points[1][i].y);
		p2.at<double>(2) = 1.0;


		
		
		cv::Mat aux = Pl.t()*(Pl*Pl.t()).inv()*p1;

		aux /= aux.at<double>(3);
		cv::Mat v1_h(3,1,CV_64F);
		
		v1_h.at<double>(0) = aux.at<double>(0) - params1.pos.at<double>(0);
		v1_h.at<double>(1) = aux.at<double>(1) - params1.pos.at<double>(1);
		v1_h.at<double>(2) = aux.at<double>(2) - params1.pos.at<double>(2);

		//cout << "v1:" << v1_h<< endl
		//	<< "aux:" << aux<< endl
		//	<< "params1.pos:" << params1.pos<< endl;
		

		aux = Pr.t()*(Pr*Pr.t()).inv()*p2;
		aux /= aux.at<double>(3);
		cv::Mat v2_h(3,1,CV_64F);
		
		v2_h.at<double>(0) = aux.at<double>(0) - params2.pos.at<double>(0);
		v2_h.at<double>(1) = aux.at<double>(1) - params2.pos.at<double>(1);
		v2_h.at<double>(2) = aux.at<double>(2) - params2.pos.at<double>(2);

		Eigen::VectorXf line1_coeff(6);
		Eigen::VectorXf line2_coeff(6);

		
		line1_coeff[0] = params1.pos.at<double>(0);
		line1_coeff[1] = params1.pos.at<double>(1);
		line1_coeff[2] = params1.pos.at<double>(2);

		line1_coeff[3] = v1_h.at<double>(0);
		line1_coeff[4] = v1_h.at<double>(1);
		line1_coeff[5] = v1_h.at<double>(2);

		line2_coeff[0] = params2.pos.at<double>(0);
		line2_coeff[1] = params2.pos.at<double>(1);
		line2_coeff[2] = params2.pos.at<double>(2);

		line2_coeff[3] = v2_h.at<double>(0);
		line2_coeff[4] = v2_h.at<double>(1);
		line2_coeff[5] = v2_h.at<double>(2);

		/*
		stringstream ss;
		ss << cnt;

		
		viewer.addLine(pcl::PointXYZ(line1_coeff[0],line1_coeff[1],line1_coeff[2]), 
						pcl::PointXYZ(line1_coeff[0] + line1_coeff[3]*10,line1_coeff[1] + line1_coeff[4]*10,line1_coeff[2]+line1_coeff[5]*10),color_r,color_g,color_b,"leftline"+ss.str());


		viewer.addLine(pcl::PointXYZ(line2_coeff[0],line2_coeff[1],line2_coeff[2]), 
						pcl::PointXYZ(line2_coeff[0] + line2_coeff[3]*10,line2_coeff[1] + line2_coeff[4]*10,line2_coeff[2]+line2_coeff[5]*10),color_r,color_g,color_b,"rightline"+ss.str());
		
		*/
		Eigen::Vector4f intersection_point;
		
		pcl::lineWithLineIntersection(line1_coeff,line2_coeff,intersection_point,0.5);

		pcl::PointXYZ p3D;

		p3D.x = intersection_point[0];
		p3D.y = intersection_point[1];
		p3D.z = intersection_point[2];

		cloud->points.push_back(p3D);

		
	}
}

cv::Mat find_intersection_ray_depth(cv::Mat p1, cam_params_t params, double depth, cv::Mat M, cv::Mat Pl) {

	cv::Mat p0(4,1,CV_64F);
	
	p0.at<double>(0) = 0;
	p0.at<double>(1) = 0;
	p0.at<double>(2) = depth;
	p0.at<double>(3) = 1;

	p0 = M.inv()*p0;

	cv::Mat p_plane(3,1,CV_64F);
	p_plane.at<double>(0) = p0.at<double>(0);
	p_plane.at<double>(1) = p0.at<double>(1);
	p_plane.at<double>(2) = p0.at<double>(2);
	cv::Mat p_line(3,1,CV_64F);
	p_line.at<double>(0) = params.pos.at<double>(0);
	p_line.at<double>(1) = params.pos.at<double>(1);
	p_line.at<double>(2) = params.pos.at<double>(2);
			
	cv::Mat n = p_plane - p_line;
	n = n / calc_norm(n,3);

	cv::Mat aux = Pl.t()*(Pl*Pl.t()).inv()*p1;
	aux /= aux.at<double>(3);
			
	//aux.at<double>(1) *=-1;


	cv::Mat l(3,1,CV_64F);
		
	l.at<double>(0) = aux.at<double>(0) - params.pos.at<double>(0);
	l.at<double>(1) = aux.at<double>(1) - params.pos.at<double>(1);
	l.at<double>(2) = aux.at<double>(2) - params.pos.at<double>(2);

	l = l / calc_norm(l,3);

			
			
	double delta = (p_plane-p_line).dot(n)/l.dot(n);
	cv::Mat depth_p = l*delta+p_line;

	return depth_p;

}

double compute_pixel_to_real_dist(int im_w, int im_h,cam_params_t params,pcl::visualization::PCLVisualizer &viewer,string name,int cam_num, double &cam_height) {
	
	cv::Mat Pl = compound_cam_transformation(params.cam,params.rot,params.trans);
	cv::Mat M =  roto_translation(params.rot,params.trans);
	cv::Mat p1(3,1,CV_64F);
	float scalef = (float)im_w/(float)640;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);

	/*
	cv::Mat origin_world(4,1,CV_64F);
	origin_world.at<double>(0) = 0;
	origin_world.at<double>(1) = 0;
	origin_world.at<double>(2) = 0;
	origin_world.at<double>(3) = 1;
	cv::Mat origin_world_cam = M*origin_world;
	*/

	p1.at<double>(0) = 0;
	p1.at<double>(1) = 0;
	p1.at<double>(2) = 1;
	cv::Mat a = find_intersection_ray_depth(p1, params, cam_height, M, Pl);
			
	p1.at<double>(0) = 1/scalef;
	p1.at<double>(1) = 0/scalef;
	p1.at<double>(2) = 1;
	cv::Mat b = find_intersection_ray_depth(p1, params, cam_height, M, Pl);
			

	//cam_height = origin_world_cam.at<double>(2);
	double diff = calc_norm(a-b,3);

	diff = 0.5*diff;
	//cout << "*****Diff:" <<diff*0.5 << endl;

	return 0.1;
	/*
	p1.at<double>(0) = depthMap.cols/(2*scalef);
	p1.at<double>(1) = depthMap.rows/(2*scalef);
	p1.at<double>(2) = 1;
	cv::Mat p = find_intersection_ray_depth(p1, params, cam_height, M, Pl);

	viewer.removeShape("sphere");
	viewer.addSphere(PointXYZ(p.at<double>(0),p.at<double>(1),p.at<double>(2)),1,"sphere");


	for (int i = 0; i < depthMap.rows;i++) {
		for (int j = 0; j < depthMap.cols;j++) {
			p1.at<double>(0) = j/scalef;
			p1.at<double>(1) = i/scalef;
			p1.at<double>(2) = 1;
			cv::Mat p = find_intersection_ray_depth(p1, params, cam_height, M, Pl);
			PointXYZ p3d;
			p3d.x = p.at<double>(0);
			p3d.y = p.at<double>(1);
			p3d.z = p.at<double>(2);
			cloud->points.push_back(p3d);
		}
	}
	*/
	/*
	viewer.removePointCloud(name);
	  viewer.addPointCloud<PointXYZ>(cloud,name);
	  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,name);
	  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)cam_colors[cam_num][0]/255.0, (float)cam_colors[cam_num][1]/255.0, (float)cam_colors[cam_num][2]/255.0,name);
	*/
}

vector<vector<PointXYZINormal> > add_surface_to_world(double depth_factor,cv::Mat depthMap, cam_params_t params,pcl::visualization::PCLVisualizer &viewer,string name,int cam_num, double cam_height, IplImage* mask,pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_aux, bool print,std::vector<std::vector<cv::Mat> > &normal_map) {
	
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<PointXYZINormal>);

	PointXYZINormal p;

	cv::Mat p1(3,1,CV_64F);
	cv::Mat p0(4,1,CV_64F);

	float scalef = (float)depthMap.cols/(float)640;

	float pixels_square_side = PIXEL3D*scalef;
		
	cv::Mat Pl = compound_cam_transformation(params.cam,params.rot,params.trans);
	cv::Mat M =  roto_translation(params.rot,params.trans);
	vector<cv::Point> cloud_pixels;
        vector<vector<PointXYZINormal> > surface_map(depthMap.rows,vector<PointXYZINormal>(depthMap.cols));
	/*
	p1.at<double>(0) = 0.0/scalef;
	p1.at<double>(1) = 0.0/scalef;
	p1.at<double>(2) = 1;
	
	cv::Mat corner1 = find_intersection_ray_depth(p1, params, cam_height, M, Pl);

	p1.at<double>(0) = depthMap.cols/scalef;
	p1.at<double>(1) = 0.0/scalef;
	p1.at<double>(2) = 1;
	
	cv::Mat corner2= find_intersection_ray_depth(p1, params, cam_height, M, Pl);

	float image_width_squares_unit  =calc_norm(corner1 - corner2,3)
	*/
	
	for (int i = 0; i < depthMap.rows; i++) {
		for (int j = 0; j < depthMap.cols; j++) {

			surface_map[i][j].x = -1000;
			surface_map[i][j].y = -1000;
			surface_map[i][j].z = -1000;

			//if (CV_IMAGE_ELEM(mask,uchar,i,j) <= 100) continue;
			if ( depthMap.at<double>((depthMap.rows-1-i)*depthMap.cols+j) <= 0) continue;
			cv::Point pixel;
			pixel.x = j;
			pixel.y = i;
			cloud_pixels.push_back(pixel);
			//double depth_val = cam_height -depthMap.at<double>((depthMap.rows-1-i)*depthMap.cols+j)/pixels_square_side;
			double depth_val = /*cam_height -*/depthMap.at<double>((depthMap.rows-1-i)*depthMap.cols+j)*depth_factor;
			p1.at<double>(0) = (double)(j)/scalef;
			p1.at<double>(1) = (double)(i)/scalef;
			p1.at<double>(2) = 1;
	
			p0.at<double>(0) = 0;
			p0.at<double>(1) = 0;
			p0.at<double>(2) = depth_val;
			p0.at<double>(3) = 1;

			p0 = M.inv()*p0;

			cv::Mat p_plane(3,1,CV_64F);
			p_plane.at<double>(0) = p0.at<double>(0);
			p_plane.at<double>(1) = p0.at<double>(1);
			p_plane.at<double>(2) = p0.at<double>(2);
			cv::Mat p_line(3,1,CV_64F);
			p_line.at<double>(0) = params.pos.at<double>(0);
			p_line.at<double>(1) = params.pos.at<double>(1);
			p_line.at<double>(2) = params.pos.at<double>(2);
			
			cv::Mat n = p_plane - p_line;
			n = n / calc_norm(n,3);

			cv::Mat aux = Pl.t()*(Pl*Pl.t()).inv()*p1;
			aux /= aux.at<double>(3);
			
			//aux.at<double>(1) *=-1;


			cv::Mat l(3,1,CV_64F);
		
			l.at<double>(0) = aux.at<double>(0) - params.pos.at<double>(0);
			l.at<double>(1) = aux.at<double>(1) - params.pos.at<double>(1);
			l.at<double>(2) = aux.at<double>(2) - params.pos.at<double>(2);

			l = l / calc_norm(l,3);

			
			
			double delta = (p_plane-p_line).dot(n)/l.dot(n);
			cv::Mat depth_p = l*delta+p_line;

		    //cv::Mat depth_p = l*depth_val +  params.pos;

			p.x = depth_p.at<double>(0);
			p.y = depth_p.at<double>(1);
			p.z = depth_p.at<double>(2);

			surface_map[i][j].x = p.x;
			surface_map[i][j].y = p.y;
			surface_map[i][j].z = p.z;
			PointXYZINormal p2;
			p2.x = p.x;
			p2.y = p.y;
			p2.z = p.z;
			//p2.normal_x = (float)CV_IMAGE_ELEM(normals,uchar,i,(j*3)+2)/255.0*2-1;
			//p2.normal_y = (float)CV_IMAGE_ELEM(normals,uchar,i,(j*3)+1)/255.0*2-1;
			//p2.normal_z = (float)CV_IMAGE_ELEM(normals,uchar,i,(j*3))/255.0*2-1;

			surface_map[i][j] = p2;
			cloud->points.push_back(p2);

		}	
	}
	/*
	// Normal estimation
	pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);
	tree->setInputCloud (cloud);
	//tree->setInputCloud (clouds[i]);
	n.setInputCloud (cloud);
	//n.setInputCloud (clouds[i]);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals_cloud);

	for (int i = 0; i < normals_cloud->size(); i++) {
	
		pcl::flipNormalTowardsViewpoint<PointXYZINormal>(cloud->points[i],params.pos.at<double>(0),
															params.pos.at<double>(1),params.pos.at<double>(2),
															normals_cloud->points[i].normal_x,normals_cloud->points[i].normal_y,normals_cloud->points[i].normal_z);
		cv::Mat nw(4,1,CV_64F);
		nw.at<double>(0) = normals_cloud->points[i].normal_x;
		nw.at<double>(1) = normals_cloud->points[i].normal_y;
		nw.at<double>(2) = normals_cloud->points[i].normal_z;
		nw.at<double>(3) = 1.0;
		cv::Mat n = M*nw;

		if (fabs(n.at<double>(2)-params.trans.at<double>(2)) >0.1) {
			normal_map[cloud_pixels[i].y][cloud_pixels[i].x].at<double>(0) = n.at<double>(0)-params.trans.at<double>(0);
			normal_map[cloud_pixels[i].y][cloud_pixels[i].x].at<double>(1) = n.at<double>(1)-params.trans.at<double>(1);
			normal_map[cloud_pixels[i].y][cloud_pixels[i].x].at<double>(2) = n.at<double>(2)-params.trans.at<double>(2);
			normal_map[cloud_pixels[i].y][cloud_pixels[i].x] = normal_map[cloud_pixels[i].y][cloud_pixels[i].x]/calc_norm(normal_map[cloud_pixels[i].y][cloud_pixels[i].x],3);
		}
			
	}
	*/
	cloud_aux = cloud;
	cout << "Adding surface"<< endl;
	if (print) {
	  viewer.removePointCloud(name);
	 // viewer.addPointCloudNormals<PointXYZINormal, Normal>(cloud,normals_cloud,1,1,name);
	  viewer.addPointCloud<PointXYZINormal>(cloud,name);
	  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,name);
	  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)cam_colors[cam_num][0]/255.0, (float)cam_colors[cam_num][1]/255.0, (float)cam_colors[cam_num][2]/255.0,name);
	}
	return surface_map;
}

vector<vector<PointXYZINormal> > add_surface_to_world(double depth_factor,cv::Mat depthMap, cam_params_t params,pcl::visualization::PCLVisualizer &viewer,string name,int cam_num, double cam_height, IplImage* mask,pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_aux, bool print,std::vector<std::vector<cv::Mat> > &normal_map,std::vector<std::vector<std::vector<matchings_t> > > matches3D, float max_proxy_dist) {
	
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<PointXYZINormal>);

	PointXYZINormal p;

	cv::Mat p1(3,1,CV_64F);
	cv::Mat p0(4,1,CV_64F);

	float scalef = (float)depthMap.cols/(float)640;

	float pixels_square_side = PIXEL3D*scalef;
		
	cv::Mat Pl = compound_cam_transformation(params.cam,params.rot,params.trans);
	cv::Mat M =  roto_translation(params.rot,params.trans);
	vector<cv::Point> cloud_pixels;
        vector<vector<PointXYZINormal> > surface_map(depthMap.rows,vector<PointXYZINormal>(depthMap.cols));
	/*
	p1.at<double>(0) = 0.0/scalef;
	p1.at<double>(1) = 0.0/scalef;
	p1.at<double>(2) = 1;
	
	cv::Mat corner1 = find_intersection_ray_depth(p1, params, cam_height, M, Pl);

	p1.at<double>(0) = depthMap.cols/scalef;
	p1.at<double>(1) = 0.0/scalef;
	p1.at<double>(2) = 1;
	
	cv::Mat corner2= find_intersection_ray_depth(p1, params, cam_height, M, Pl);

	float image_width_squares_unit  =calc_norm(corner1 - corner2,3)
	*/
	
	for (int i = 0; i < depthMap.rows; i++) {
		for (int j = 0; j < depthMap.cols; j++) {

			surface_map[i][j].x = -1000;
			surface_map[i][j].y = -1000;
			surface_map[i][j].z = -1000;

			if (matches3D[i][j].size() == 0) continue;

			//if (CV_IMAGE_ELEM(mask,uchar,i,j) <= 100) continue;
			if ( depthMap.at<double>((depthMap.rows-1-i)*depthMap.cols+j) <= 0) continue;
			cv::Point pixel;
			pixel.x = j;
			pixel.y = i;
			cloud_pixels.push_back(pixel);
			//double depth_val = cam_height -depthMap.at<double>((depthMap.rows-1-i)*depthMap.cols+j)/pixels_square_side;
			double depth_val = /*cam_height -*/depthMap.at<double>((depthMap.rows-1-i)*depthMap.cols+j)*depth_factor;
			p1.at<double>(0) = (double)(j)/scalef;
			p1.at<double>(1) = (double)(i)/scalef;
			p1.at<double>(2) = 1;
	
			p0.at<double>(0) = 0;
			p0.at<double>(1) = 0;
			p0.at<double>(2) = depth_val;
			p0.at<double>(3) = 1;

			p0 = M.inv()*p0;

			cv::Mat p_plane(3,1,CV_64F);
			p_plane.at<double>(0) = p0.at<double>(0);
			p_plane.at<double>(1) = p0.at<double>(1);
			p_plane.at<double>(2) = p0.at<double>(2);
			cv::Mat p_line(3,1,CV_64F);
			p_line.at<double>(0) = params.pos.at<double>(0);
			p_line.at<double>(1) = params.pos.at<double>(1);
			p_line.at<double>(2) = params.pos.at<double>(2);
			
			cv::Mat n = p_plane - p_line;
			n = n / calc_norm(n,3);

			cv::Mat aux = Pl.t()*(Pl*Pl.t()).inv()*p1;
			aux /= aux.at<double>(3);
			
			//aux.at<double>(1) *=-1;


			cv::Mat l(3,1,CV_64F);
		
			l.at<double>(0) = aux.at<double>(0) - params.pos.at<double>(0);
			l.at<double>(1) = aux.at<double>(1) - params.pos.at<double>(1);
			l.at<double>(2) = aux.at<double>(2) - params.pos.at<double>(2);

			l = l / calc_norm(l,3);

			
			
			double delta = (p_plane-p_line).dot(n)/l.dot(n);
			cv::Mat depth_p = l*delta+p_line;

		    //cv::Mat depth_p = l*depth_val +  params.pos;

			p.x = depth_p.at<double>(0);
			p.y = depth_p.at<double>(1);
			p.z = depth_p.at<double>(2);

			surface_map[i][j].x = p.x;
			surface_map[i][j].y = p.y;
			surface_map[i][j].z = p.z;
			PointXYZINormal p2;
			p2.x = p.x;
			p2.y = p.y;
			p2.z = p.z;
			//p2.normal_x = (float)CV_IMAGE_ELEM(normals,uchar,i,(j*3)+2)/255.0*2-1;
			//p2.normal_y = (float)CV_IMAGE_ELEM(normals,uchar,i,(j*3)+1)/255.0*2-1;
			//p2.normal_z = (float)CV_IMAGE_ELEM(normals,uchar,i,(j*3))/255.0*2-1;

			
			PointXYZ p_int = matches3D[i][j][0].point;
			float dist = sqrt((double)((p2.x - p_int.x)*(p2.x - p_int.x) + (p2.y - p_int.y)*(p2.y - p_int.y) + (p2.z - p_int.z)*(p2.z - p_int.z)));
			if (dist < max_proxy_dist) {
				cloud->points.push_back(p2);
				surface_map[i][j] = p2;
			}

		}	
	}
	/*
	// Normal estimation
	pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal>);
	tree->setInputCloud (cloud);
	//tree->setInputCloud (clouds[i]);
	n.setInputCloud (cloud);
	//n.setInputCloud (clouds[i]);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals_cloud);

	for (int i = 0; i < normals_cloud->size(); i++) {
	
		pcl::flipNormalTowardsViewpoint<PointXYZINormal>(cloud->points[i],params.pos.at<double>(0),
															params.pos.at<double>(1),params.pos.at<double>(2),
															normals_cloud->points[i].normal_x,normals_cloud->points[i].normal_y,normals_cloud->points[i].normal_z);
		cv::Mat nw(4,1,CV_64F);
		nw.at<double>(0) = normals_cloud->points[i].normal_x;
		nw.at<double>(1) = normals_cloud->points[i].normal_y;
		nw.at<double>(2) = normals_cloud->points[i].normal_z;
		nw.at<double>(3) = 1.0;
		cv::Mat n = M*nw;

		if (fabs(n.at<double>(2)-params.trans.at<double>(2)) >0.1) {
			normal_map[cloud_pixels[i].y][cloud_pixels[i].x].at<double>(0) = n.at<double>(0)-params.trans.at<double>(0);
			normal_map[cloud_pixels[i].y][cloud_pixels[i].x].at<double>(1) = n.at<double>(1)-params.trans.at<double>(1);
			normal_map[cloud_pixels[i].y][cloud_pixels[i].x].at<double>(2) = n.at<double>(2)-params.trans.at<double>(2);
			normal_map[cloud_pixels[i].y][cloud_pixels[i].x] = normal_map[cloud_pixels[i].y][cloud_pixels[i].x]/calc_norm(normal_map[cloud_pixels[i].y][cloud_pixels[i].x],3);
		}
			
	}
	*/
	cloud_aux = cloud;
	cout << "Adding surface"<< endl;
	if (print) {
	  viewer.removePointCloud(name);
	 // viewer.addPointCloudNormals<PointXYZINormal, Normal>(cloud,normals_cloud,1,1,name);
	  viewer.addPointCloud<PointXYZINormal>(cloud,name);
	  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,name);
	  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)cam_colors[cam_num][0]/255.0, (float)cam_colors[cam_num][1]/255.0, (float)cam_colors[cam_num][2]/255.0,name);
	}
	return surface_map;
}



vector<vector<PointXYZINormal> > add_surface_to_world_new(double depth_factor,cv::Mat depthMap, cam_params_t params,pcl::visualization::PCLVisualizer &viewer,string name,int cam_num, double cam_height, IplImage* mask,pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_aux, bool print,IplImage* normals) {
	
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<PointXYZINormal>);

	PointXYZINormal p;

	cv::Mat p1(3,1,CV_64F);
	cv::Mat p0(4,1,CV_64F);

	float scalef = (float)depthMap.cols/(float)640;

	float pixels_square_side = PIXEL3D*scalef;
		
	cv::Mat Pl = compound_cam_transformation(params.cam,params.rot,params.trans);
	cv::Mat M =  roto_translation(params.rot,params.trans);
	
        vector<vector<PointXYZINormal> > surface_map(depthMap.rows,vector<PointXYZINormal>(depthMap.cols));


	p1.at<double>(0) = depthMap.cols/(2*scalef);
	p1.at<double>(1) = depthMap.rows/(2*scalef);
	p1.at<double>(2) = 1;
	
	cv::Mat middle_base_point = find_intersection_ray_depth(p1, params, cam_height, M, Pl);
	cv::Mat base_normal = params.pos - middle_base_point;
	base_normal = base_normal / calc_norm(base_normal,3);
	/*
	p1.at<double>(0) = 0.0/scalef;
	p1.at<double>(1) = 0.0/scalef;
	p1.at<double>(2) = 1;
	
	cv::Mat corner1 = find_intersection_ray_depth(p1, params, cam_height, M, Pl);

	p1.at<double>(0) = depthMap.cols/scalef;
	p1.at<double>(1) = 0.0/scalef;
	p1.at<double>(2) = 1;
	
	cv::Mat corner2= find_intersection_ray_depth(p1, params, cam_height, M, Pl);

	float image_width_squares_unit  =calc_norm(corner1 - corner2,3)
	*/
	for (int i = 0; i < depthMap.rows; i++) {
		for (int j = 0; j < depthMap.cols; j++) {

			surface_map[i][j].x = -1000;
			surface_map[i][j].y = -1000;
			surface_map[i][j].z = -1000;

			//if (CV_IMAGE_ELEM(mask,uchar,i,j) <= 100) continue;
			if ( depthMap.at<double>((depthMap.rows-1-i)*depthMap.cols+j) <= 0) continue;

			p1.at<double>(0) = j/scalef;
			p1.at<double>(1) = i/scalef;
			p1.at<double>(2) = 1;
	
			cv::Mat base_point = find_intersection_ray_depth(p1, params, cam_height, M, Pl);

			cv::Mat surface_point = base_point + base_normal*depthMap.at<double>((depthMap.rows-1-i)*depthMap.cols+j)*depth_factor;

			p.x = surface_point.at<double>(0);
			p.y = surface_point.at<double>(1);
			p.z = surface_point.at<double>(2);
			surface_map[i][j].x = p.x;
			surface_map[i][j].y = p.y;
			surface_map[i][j].z = p.z;
			PointXYZINormal p2;
			p2.x = p.x;
			p2.y = p.y;
			p2.z = p.z;
			p2.normal_x = (float)CV_IMAGE_ELEM(normals,uchar,i,(j*3)+2)/255.0*2-1;
			p2.normal_y = (float)CV_IMAGE_ELEM(normals,uchar,i,(j*3)+1)/255.0*2-1;
			p2.normal_z = (float)CV_IMAGE_ELEM(normals,uchar,i,(j*3))/255.0*2-1;

			surface_map[i][j] = p2;
			cloud->points.push_back(p2);

		}	
	}
	cloud_aux = cloud;
	cout << "Adding surface"<< endl;
	if (print) {
	  viewer.removePointCloud(name);
	  viewer.addPointCloud<PointXYZINormal>(cloud,name);
	  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,name);
	  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (float)cam_colors[cam_num][0]/255.0, (float)cam_colors[cam_num][1]/255.0, (float)cam_colors[cam_num][2]/255.0,name);
	}
	return surface_map;
}



void acc_matchings(int im_w, int im_h,vector<vector<cv::Point> > pixel_matches, vector<vector<vector<vector<matchings_t> > > > &accumulator,cam_params_t params1, cam_params_t params2,int cam1, int cam2) {

	cv::Mat Pl = compound_cam_transformation(params1.cam,params1.rot,params1.trans);
	cv::Mat Pr = compound_cam_transformation(params2.cam,params2.rot,params2.trans);
		
		//cout << "Pl: " << Pl << endl;
		//cout << "Pr: " << Pr << endl;

		
	float scalef = (float)im_w/640.0;

	for (int i = 0; i < pixel_matches[0].size(); i++) {

		
		cv::Mat p1(3,1,CV_64F);
		cv::Mat p2(3,1,CV_64F);
		
		/*cout << "---------------------"<<endl;
		cout << pixel_matches[0][i] << endl;
		cout << pixel_matches[1][i] << endl;
		cout << "---------------------"<<endl;
		*/

		p1.at<double>(0) = (double)(pixel_matches[0][i].x/scalef);
		p1.at<double>(1) = (double)(pixel_matches[0][i].y/scalef);
		p1.at<double>(2) = 1.0;

		p2.at<double>(0) = (double)(pixel_matches[1][i].x/scalef);
		p2.at<double>(1) = (double)(pixel_matches[1][i].y/scalef);
		p2.at<double>(2) = 1.0;


		
		
		cv::Mat aux = Pl.t()*(Pl*Pl.t()).inv()*p1;

		aux /= aux.at<double>(3);
		cv::Mat v1_h(3,1,CV_64F);
		
		v1_h.at<double>(0) = aux.at<double>(0) - params1.pos.at<double>(0);
		v1_h.at<double>(1) = aux.at<double>(1) - params1.pos.at<double>(1);
		v1_h.at<double>(2) = aux.at<double>(2) - params1.pos.at<double>(2);

		//cout << "v1:" << v1_h<< endl
		//	<< "aux:" << aux<< endl
		//	<< "params1.pos:" << params1.pos<< endl;
		

		aux = Pr.t()*(Pr*Pr.t()).inv()*p2;
		aux /= aux.at<double>(3);
		cv::Mat v2_h(3,1,CV_64F);
		
		v2_h.at<double>(0) = aux.at<double>(0) - params2.pos.at<double>(0);
		v2_h.at<double>(1) = aux.at<double>(1) - params2.pos.at<double>(1);
		v2_h.at<double>(2) = aux.at<double>(2) - params2.pos.at<double>(2);

		Eigen::VectorXf line1_coeff(6);
		Eigen::VectorXf line2_coeff(6);

		
		line1_coeff[0] = params1.pos.at<double>(0);
		line1_coeff[1] = params1.pos.at<double>(1);
		line1_coeff[2] = params1.pos.at<double>(2);

		line1_coeff[3] = v1_h.at<double>(0);
		line1_coeff[4] = v1_h.at<double>(1);
		line1_coeff[5] = v1_h.at<double>(2);

		line2_coeff[0] = params2.pos.at<double>(0);
		line2_coeff[1] = params2.pos.at<double>(1);
		line2_coeff[2] = params2.pos.at<double>(2);

		line2_coeff[3] = v2_h.at<double>(0);
		line2_coeff[4] = v2_h.at<double>(1);
		line2_coeff[5] = v2_h.at<double>(2);

		
		Eigen::Vector4f intersection_point;
		
		//if(!pcl::lineWithLineIntersection(line1_coeff,line2_coeff,intersection_point,0.5))
			//cout << "No intersection" << endl;
		pcl::lineWithLineIntersection(line1_coeff,line2_coeff,intersection_point,0.5);
		pcl::PointXYZ p3D;

		p3D.x = intersection_point[0];
		p3D.y = intersection_point[1];
		p3D.z = intersection_point[2];

		matchings_t match;
		if (cam1==cam2) {
			cout << "ERROR: " << cam1 << " " << cam2 << endl;
			exit(0);
		}
		match.paired_cam = cam2;
		match.paired_pixel = cv::Point(pixel_matches[1][i].x,pixel_matches[1][i].y);
		match.point = p3D;
		//cout << p3D << endl;
		accumulator[cam1][pixel_matches[0][i].y][pixel_matches[0][i].x].push_back(match);

		match.paired_cam = cam1;
		match.paired_pixel = cv::Point(pixel_matches[0][i].x,pixel_matches[0][i].y);
		accumulator[cam2][pixel_matches[1][i].y][pixel_matches[1][i].x].push_back(match);
		
	}

}

void add_matchings_to_world(std::vector<std::vector<std::vector<std::vector<matchings_t> > > > matches3D,pcl::visualization::PCLVisualizer &viewer) {


	std:
	int height = matches3D[0].size();
	int width = matches3D[0][0].size();
	
	for (int cam_idx = 0; cam_idx < CAM_NUM; cam_idx++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				for (int k = 0; k < matches3D[cam_idx][i][j].size(); k++) {
					cloud->points.push_back(matches3D[cam_idx][i][j][k].point);
				}	
			}
		}
		stringstream ss;
		ss << cam_idx;
		string cloud_name = "cloud_matches" + ss.str();
		viewer.addPointCloud(cloud,cloud_name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,cloud_name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,(float)cam_colors[cam_idx][0]/255.0,(float)cam_colors[cam_idx][1]/255,(float)cam_colors[cam_idx][2]/255.0,cloud_name);
	}
}

void visualize_light_dirs(vector<cam_params_t> params, pcl::visualization::PCLVisualizer &viewer, int cam_num, cv::Mat l_dirs) {

	cv::Mat p_plane(3,1,CV_64F);
	p_plane.at<double>(0) = 0;
	p_plane.at<double>(1) = 0;
	p_plane.at<double>(2) = 0;
	
	cv::Mat p_line(3,1,CV_64F);
	p_line.at<double>(0) = params[cam_num].pos.at<double>(0);
	p_line.at<double>(1) = params[cam_num].pos.at<double>(1);
	p_line.at<double>(2) = params[cam_num].pos.at<double>(2);
			
	cv::Mat n(3,1,CV_64F);
	n.at<double>(0) = 0;
	n.at<double>(1) = 0;
	n.at<double>(2) = 1;

	cv::Mat aux(4,1,CV_64F);
	aux.at<double>(0) = 0;
	aux.at<double>(1) = 0;
	aux.at<double>(2) = 1;
	aux.at<double>(3) = 1;

	cv::Mat M = roto_translation(params[cam_num].rot, params[cam_num].trans);

	aux = M.inv()*aux;

	cv::Mat l(3,1,CV_64F);
	l.at<double>(0) = aux.at<double>(0);
	l.at<double>(1) = aux.at<double>(1);
	l.at<double>(2) = aux.at<double>(2);
	
	l -= params[cam_num].pos;
	l = l / calc_norm(l,3);

	double delta = (p_plane-p_line).dot(n)/l.dot(n);
	cv::Mat depth_p = l*delta+p_line;

	PointXYZ p;
	p.x = depth_p.at<double>(0);
	p.y = depth_p.at<double>(1);
	p.z = depth_p.at<double>(2);

	viewer.addSphere(p,1,1,0,0,"sphere");

}

void show_normals_world(vector<vector<cv::Mat> > normals,pcl::visualization::PCLVisualizer &viewer) {
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<Normal>);
	int h = normals.size();
	int w = normals[0].size();
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			cloud->points.push_back(PointXYZ(i,j,0));
			normal_cloud->points.push_back(Normal(normals[i][j].at<double>(0),normals[i][j].at<double>(1),normals[i][j].at<double>(2)));
		}
	}
	viewer.removeAllPointClouds();
	viewer.addPointCloudNormals<PointXYZ,Normal>(cloud,normal_cloud,1,1,"normals");


}

void avg_clouds_color(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) {

	output_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
	for (int j = 0; j < CAM_NUM; j++) {
			//if (i==j) continue;
			*output_cloud += *clouds[j];
		}
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(output_cloud);
	std::vector<int> nIdx;
	std::vector<float> nDist;
	for (int i = 0; i < output_cloud->points.size(); i++) {
		tree->radiusSearch(output_cloud->points[i],0.5,nIdx,nDist);
		float r = 0,g = 0,b= 0;
		int cnt=0;
		PointXYZRGB p;
		for (int k = 0; k < nIdx.size();k++) {
			r+=output_cloud->points[nIdx[k]].r;
			g+=output_cloud->points[nIdx[k]].g;
			b+=output_cloud->points[nIdx[k]].b;
			++cnt;
		}
		output_cloud->points[i].r = r/(float)cnt;
		output_cloud->points[i].g = g/(float)cnt;
		output_cloud->points[i].b = b/(float)cnt;
	}
}

void filter_clouds(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud) {


	output_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
	for (int i = 0; i < CAM_NUM; i++) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_clouds(new pcl::PointCloud<PointXYZRGB>);
		for (int j = 0; j < CAM_NUM; j++) {
			if (i==j) continue;
			*all_clouds += *clouds[j];
		}
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud(all_clouds);
		std::vector<int> nIdx;
		std::vector<float> nDist;
		for (int j = 0; j < clouds[i]->points.size(); j++) {
			tree->radiusSearch(clouds[i]->points[j],0.5,nIdx,nDist);
			if (nDist.size()>0 && nDist[0] < 0.25) {
				float r = 0,g = 0,b= 0;
				int cnt=0;
				PointXYZRGB p;
				for (int k = 0; k < nIdx.size();k++) {
					r+=all_clouds->points[nIdx[k]].r;
					g+=all_clouds->points[nIdx[k]].g;
					b+=all_clouds->points[nIdx[k]].b;
					++cnt;
				}
				p.x = clouds[i]->points[j].x;
				p.y = clouds[i]->points[j].y;
				p.z = clouds[i]->points[j].z;
				p.r = r/(float)cnt;
				p.g = g/(float)cnt;
				p.b = b/(float)cnt;
				
				output_cloud->points.push_back(p);
			}
		}
	}

}
void add_coloured_meshes(vector <IplImage*> ims, vector<vector<vector<PointXYZINormal> > > surfaces, vector<cam_params_t>& params,pcl::visualization::PCLVisualizer &viewer, float final_smooth_r) {

	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds(CAM_NUM);
	//vector<pcl::PointCloud<pcl::Normal>::Ptr> normals(CAM_NUM);
        vector<vector<cv::Point> > clouds_pixel_idx(CAM_NUM);
	for (int i = 0; i< surfaces.size(); i++) {
		clouds[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
		//normals[i] = pcl::PointCloud<Normal>::Ptr(new pcl::PointCloud<Normal>);
		for (int u = 0; u< surfaces[i].size(); u++) {
			for (int v = 0; v< surfaces[i][0].size(); v++) {
				if (surfaces[i][u][v].x == -1000) continue;
				PointXYZRGB p3d;
				p3d.x = surfaces[i][u][v].x;
				p3d.y = surfaces[i][u][v].y;
				p3d.z = surfaces[i][u][v].z;
				p3d.r = CV_IMAGE_ELEM(ims[i],uchar,u,v*3 +2);
				p3d.g = CV_IMAGE_ELEM(ims[i],uchar,u,v*3 +1);
				p3d.b = CV_IMAGE_ELEM(ims[i],uchar,u,v*3 );
				clouds[i]->points.push_back(p3d);
				cv::Point p;
				p.x = v;
				p.y = u;
				clouds_pixel_idx[i].push_back(p);
				//normals[i]->points.push_back(pcl::Normal(surfaces[i][u][v].normal_x,surfaces[i][u][v].normal_y,surfaces[i][u][v].normal_z));
			}
		}
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_clouds(new pcl::PointCloud<PointXYZRGB>);
	/*
	for (int j = 0; j < CAM_NUM; j++) {		
			*all_clouds += *clouds[j];
	}
	*/
	/*
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (all_clouds);
	//tree->setInputCloud (clouds[i]);
	n.setInputCloud (all_clouds);
	//n.setInputCloud (clouds[i]);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	 pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr poisson_tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields (*all_clouds, *normals, *cloud_with_normals);

	pcl::Poisson<pcl::PointXYZRGBNormal>poi;
   poi.setInputCloud(cloud_with_normals); 
   poi.setSearchMethod(poisson_tree); 
   
   pcl::PolygonMesh triangles;
   poi.reconstruct(triangles);

   */



	
	avg_clouds_color(clouds,  all_clouds); 

	
	/*
	for (int i = 0; i < CAM_NUM;i++) {
		*all_clouds = *all_clouds + *clouds[i];
	}
	*/

	
		 // Create a KD-Tree
	  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr mls_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);

	

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
 
	 mls.setComputeNormals (false);

	// Set parameters
	mls.setInputCloud (all_clouds);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (mls_tree);
	mls.setSearchRadius (final_smooth_r);

	// Reconstruct
	pcl::PolygonMesh triangles;
	mls.process (*mls_points);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	
	
	sor.setInputCloud (mls_points);
	sor.setLeafSize (0.1, 0.1, 0.1);
	
	sor.filter (*sampled_mls_points);
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);

	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr chull_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	//chull_tree->setInputCloud(sampled_mls_points);

	pcl::ConcaveHull<pcl::PointXYZRGB> chull;
	chull.setInputCloud (sampled_mls_points);
	//chull.setSearchMethod(chull_tree);
	chull.setAlpha(0.2);
	chull.setKeepInformation(true);
	
	//chull.reconstruct (*cloud_hull);
	
	chull.reconstruct(triangles);
	
	
	/*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	
	
	sor.setInputCloud (mls_points);
	sor.setLeafSize (0.25, 0.25, 0.25);
	
	sor.filter (*sampled_mls_points);
	

	//sampled_mls_points = mls_points;

	//for (int i= 0; i < CAM_NUM; i++) {
	
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (sampled_mls_points);
	//tree->setInputCloud (clouds[i]);
	n.setInputCloud (sampled_mls_points);
	//n.setInputCloud (clouds[i]);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields (*sampled_mls_points, *normals, *cloud_with_normals);
    //pcl::concatenateFields (*clouds[i], *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (3);

	// Set typical values for the parameters
	 gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	 gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	 gp3.setMinimumAngle(M_PI/18); // 10 degrees
	 gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	 gp3.setNormalConsistency(true);

  // Get result
	 gp3.setInputCloud (cloud_with_normals);
	 gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);
	//stringstream ss;
	//ss<<i;
	*/
	
	viewer.addPolygonMesh(triangles,"smoothSurf");
	viewer.addPointCloud(sampled_mls_points,"mls_points");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"mls_points");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,1,"mls_points");
	//}

}
