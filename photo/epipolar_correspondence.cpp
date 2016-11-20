
#include "epipolar_correspondence.h"


#define REAL_IM_WIDTH 640
#define REAL_IM_HEIGHT 480
#define LINE_RES 50

//using namespace cv;
using namespace std;

vector<cv::Point> find_mask_intersections(double a, double b, double c, IplImage* mask) {



	bool flag = false;
	int y;
	cv::Point p1,p2;

	int im_width = mask->width;
	int im_height = mask->height;
	vector<cv::Point> intersections;

	float m = -a/b;

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
	else {
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

vector<vector<cv::Point>> find_epipolar_lines(cv::Mat E, int im_point_x, int im_point_y, cv::Mat Al, cv::Mat Ar, IplImage *im1, IplImage *im2,IplImage *mask1, IplImage *mask2, int line_num) {

	
	//cvNamedWindow("Image1");
	//cvNamedWindow("Image2");

	

	CvScalar color = cvScalar(0,0,0);//rand()%255,rand()%255,rand()%255);
		

	vector<vector<cv::Point>> lines(2);

	float x_scale_f = (float)mask1->width/(float)REAL_IM_WIDTH;
	float y_scale_f = (float)mask1->height/(float)REAL_IM_HEIGHT;

	im_point_x = im_point_x/x_scale_f;
	im_point_y = im_point_y/y_scale_f;

	cv::Mat im_point(3,1,CV_64F);
	double* im_point_data = (double*)(im_point.data);
	im_point_data[0] = im_point_x;
	im_point_data[1] = im_point_y;
	im_point_data[2] = 1.0;

	cv::Mat film_point = Al.inv()*im_point;
	
	cv::Mat l = E*film_point;
	double *l_data = (double *)(l.data);
	double a = l_data[0];
	double b = l_data[1];
	double c = l_data[2];

	double x = -0.7;
	double y = (-c-a*x)/b;
	cv::Mat aux_film_point(3,1,CV_64F);
	double * aux_film_point_data = (double*)(aux_film_point.data);
	aux_film_point_data[0] = x;
	aux_film_point_data[1] = y;
	aux_film_point_data[2] = 1.0;

	cv::Mat p1 = Ar*aux_film_point;
	cv::Mat p1_aux(3,1,CV_64F);
	
	p1_aux.at<double>(0)= p1.at<double>(0);
	p1_aux.at<double>(1)= p1.at<double>(1);
	p1_aux.at<double>(2)= 1;

	double* p1_data = (double*)(p1.data);
	
	p1_data[0] = p1_data[0]*x_scale_f;
	p1_data[1] = p1_data[1]*y_scale_f;
	
	x = 0.7;
	y = (-c-a*x)/b;
	aux_film_point_data[0] = x;
	aux_film_point_data[1] = y;
	aux_film_point_data[2] = 1.0;

	cv::Mat p2 = Ar*aux_film_point;
	double* p2_data = (double*)(p2.data);

	p2_data[0] = p2_data[0]*x_scale_f;
	p2_data[1] = p2_data[1]*y_scale_f;

	a = p2_data[1]-p1_data[1];
	b = -p2_data[0]+p1_data[0];
	c = p1_data[1]*p2_data[0]-p1_data[0]*p2_data[1];

	
	vector<cv::Point> intersections = find_mask_intersections(a,b,c,mask2);
	
	if (intersections.size() < 2) 
		return lines;

		//cout << intersections[0] << endl
		//<< intersections[1] << endl;

	

	cv::Point im_p1 = intersections[0];
	cv::Point im_p2 = intersections[1];

	stringstream ss;
	ss << line_num;
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4, 0.4);
	string line_number = ss.str();
	
	cvLine(im2,im_p1*(1/x_scale_f),im_p2*(1/x_scale_f),color,1);
	cvPutText(im2, line_number.c_str(),im_p1*(1/x_scale_f),&font,cvScalar(0,0,255));					

	float line_length = sqrt(double((im_p1.x - im_p2.x)*(im_p1.x - im_p2.x) + (im_p1.y - im_p2.y)*(im_p1.y - im_p2.y)));
	
	float line_dir_x = (im_p2.x - im_p1.x)/line_length;
	float line_dir_y = (im_p2.y - im_p1.y)/line_length;
	
	
	float step = line_length/LINE_RES;

	
	
	cv::Point p;
	float dist = 0;
	while (dist < line_length) {
		p.x = im_p1.x+line_dir_x*dist;
		p.y = im_p1.y+line_dir_y*dist;
		lines[1].push_back(p);
		dist += step;
		//cvCircle(im2,p,2,cvScalar(255),2);
	}
	/*
	vector<Point> segment_on_mask;
	for (int i = 0; i < lines[0].size(); i++) {
		if (CV_IMAGE_ELEM(mask2,uchar,lines[0][i].y,lines[0][i].x) > 100) {
			segment_on_mask.push_back(lines[0][i]);
			cvCircle(im2,lines[0][i],2,cvScalar(255),2);
		}
	}
	lines[0] = segment_on_mask;
	*/
	
	//line in the first camera frame
	film_point = Ar.inv()*p1_aux;

	
	l = E.t()*film_point;

	a = l_data[0];
	b = l_data[1];
	c = l_data[2];

	

	x = -0.7;
	y = (-c-a*x)/ b;
	aux_film_point_data[0] = x;
	aux_film_point_data[1] = y;
	aux_film_point_data[2] = 1.0;
	p1 = Al*aux_film_point;

	p1_data[0] = p1_data[0]*x_scale_f;
	p1_data[1] = p1_data[1]*y_scale_f;

	x = 0.7;
	y = (-c-a*x)/ b;
	aux_film_point_data[0] = x;
	aux_film_point_data[1] = y;
	aux_film_point_data[2] = 1.0;
	p2 = Al*aux_film_point;

	p2_data[0] = p2_data[0]*x_scale_f;
	p2_data[1] = p2_data[1]*y_scale_f;

	a = p2_data[1]-p1_data[1];
	b = -p2_data[0]+p1_data[0];
	c = p1_data[1]*p2_data[0]-p1_data[0]*p2_data[1];

	intersections = find_mask_intersections(a,b,c,mask1);

	if (intersections.size() < 2) 
		return lines;

	//cout << intersections[0] << endl
	//	<< intersections[1] << endl;

	

	im_p1 = intersections[0];
	im_p2 = intersections[1];

	cvLine(im1,im_p1*(1/x_scale_f),im_p2*(1/x_scale_f),color,1);
	cvPutText(im1, line_number.c_str(),im_p1*(1/x_scale_f),&font,cvScalar(0,0,255));	

	line_length = sqrt(double((im_p1.x - im_p2.x)*(im_p1.x - im_p2.x) + (im_p1.y - im_p2.y)*(im_p1.y - im_p2.y)));
	
	line_dir_x = (im_p2.x - im_p1.x)/line_length;
	line_dir_y = (im_p2.y - im_p1.y)/line_length;
	
	
	//step = line_length/line_res;

	
	dist = 0;
	while (dist < line_length) {
		p.x = im_p1.x+line_dir_x*dist;
		p.y = im_p1.y+line_dir_y*dist;
		lines[0].push_back(p);
		dist += step;
	//	cvCircle(im1,p,2,cvScalar(255),2);
		
	}
	/*
	for (int i = 0; i < lines[1].size(); i++) {
		if (CV_IMAGE_ELEM(mask1,uchar,lines[1][i].y,lines[1][i].x) > 100) {
			segment_on_mask.push_back(lines[1][i]);
			cvCircle(im1,lines[1][i],2,cvScalar(255),2);
		}
	}
	lines[1] = segment_on_mask;
	*/
	//cvShowImage("Image1",im1);
	//cvShowImage("Image2",im2);
	//cvWaitKey();

	return lines;
}
