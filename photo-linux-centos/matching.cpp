#include "matching.h"
#include "utils.h"

#define ALPHA 0.5
#define BETA 1.0
#define GAMMA 0.25
#define BETA_P 1.0

#define INF 9999999999

//using namespace cv;
//using namespace std;

std::vector<std::vector<double> >  clo;
std::vector<std::vector<double> >  clm;
std::vector<std::vector<double> >  cro;
std::vector<std::vector<double> >  crm;
std::vector<std::vector<double> >  bl_lo;
std::vector<std::vector<double> >  bl_lm;
std::vector<std::vector<double> >  bl_ro;
std::vector<std::vector<double> >  bl_rm;

std::vector<std::vector<double> >  costs;


IplImage* visualize_normals_world (std::vector<std::vector<cv::Mat> >  n1, cam_params_t params,std::string win_name1,std::string win_name2) {
	//show normals

	cv::Mat M = roto_translation(params.rot, params.trans);
	cv::Mat pos = params.pos;
	
	IplImage* normal_im = cvCreateImage(cvSize(n1[0].size(),n1.size()),IPL_DEPTH_8U,3);
	/*
	for (int i = 0; i < n1.size(); i++) {
		for (int j = 0; j < n1[0].size(); j++) {
			CV_IMAGE_ELEM(normal_im, uchar, i, (j * 3)) = (unsigned char)((n1[i][j].at<double>(2)+1)/2*255);
			CV_IMAGE_ELEM(normal_im, uchar, i, (j * 3)+1) = (unsigned char)((n1[i][j].at<double>(1)+1)/2*255);
			CV_IMAGE_ELEM(normal_im, uchar, i, (j * 3)+2) = (unsigned char)((n1[i][j].at<double>(0)+1)/2*255);
			
		}
	}
	cvNamedWindow(win_name1.c_str());
	cvShowImage(win_name1.c_str(),normal_im);
	*/

	cv::Mat nw(4,1,CV_64F);
	for (int i = 0; i < n1.size(); i++) {
		for (int j = 0; j < n1[0].size(); j++) {

			nw.at<double>(0) = n1[i][j].at<double>(0);
			nw.at<double>(1) = n1[i][j].at<double>(1);
			nw.at<double>(2) = n1[i][j].at<double>(2);
			nw.at<double>(3) = 1.0;
			nw = M.inv()*nw;

			nw.at<double>(0) -= pos.at<double>(0);
			nw.at<double>(1) -= pos.at<double>(1);
			nw.at<double>(2) -= pos.at<double>(2);
			nw = nw /calc_norm(nw,3);
			
			CV_IMAGE_ELEM(normal_im, uchar, i, (j * 3)) = (unsigned char)((nw.at<double>(2)+1)/2*255);
			CV_IMAGE_ELEM(normal_im, uchar, i, (j * 3)+1) = (unsigned char)((nw.at<double>(1)+1)/2*255);
			CV_IMAGE_ELEM(normal_im, uchar, i, (j * 3)+2) = (unsigned char)((nw.at<double>(0)+1)/2*255);
			
		}
	}
	/*
	cvNamedWindow(win_name2.c_str());
	cvShowImage(win_name2.c_str(),normal_im);

	cvWaitKey();
	*/
	return normal_im;

}

void visualize_matching_cost(std::vector<std::vector<double> >  cost) {

	IplImage* cost_im = cvCreateImage(cvSize(cost[0].size(),cost.size()),IPL_DEPTH_8U,1);
	IplImage* cost_im_big = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	
	for (int i = 0; i < cost.size(); i++) {
		for (int j = 0; j < cost[0].size(); j++) {
			CV_IMAGE_ELEM(cost_im, uchar, i, j ) = (unsigned char)((cost[i][j]*255));
		}
	}

	cvResize(cost_im,cost_im_big,0);
	cvNamedWindow("matching cost");
	cvShowImage("matching cost",cost_im_big);
	cvWaitKey();

}

std::vector<std::vector<double> >  calc_matching_cost(std::vector<std::vector<cv::Point> >  lines,std::vector<std::vector<cv::Mat> >  n1,std::vector<std::vector<cv::Mat> >  n2,cam_params_t cam1,cam_params_t cam2) {
	
	std::vector<cv::Point> l = lines[0];
	std::vector<cv::Point> r = lines[1];

	//float fscale = (float)n1[0].size()/640.0;

	

	std::vector<std::vector<double> >  cost(r.size(),std::vector<double>(l.size(),1));

	cv::Mat Mw1 = roto_translation(cam1.rot, cam1.trans);
	cv::Mat Mw2 = roto_translation(cam2.rot, cam2.trans);
	for (int i = 0; i < l.size(); i++) {
		cv::Mat nl1 = n1[l[i].y][l[i].x];		
		cv::Mat nl1_h(4,1,CV_64F);

		nl1_h.at<double>(0) = nl1.at<double>(0);
		nl1_h.at<double>(1) = nl1.at<double>(1);
		nl1_h.at<double>(2) = nl1.at<double>(2);
		nl1_h.at<double>(3) = 1.0;

		cv::Mat nlw_h = Mw1.inv()*nl1_h;
		nlw_h.at<double>(0) -= cam1.pos.at<double>(0);
		nlw_h.at<double>(1) -= cam1.pos.at<double>(1);
		nlw_h.at<double>(2) -= cam1.pos.at<double>(2);
		nlw_h = nlw_h/calc_norm(nlw_h,3);

		int top;
		if (r.size()-1 < i)
			top = r.size()-1;
		else
			top = i;
		
		for (int j = 0; j <= top; j++) {
			/*std::cout << n2[r[j].y][r[j].x] << std::endl
				<< n1[l[i].y][l[i].x]	<<std::endl
				<< "-------------------" << std::endl;
				*/
			cv::Mat nr2 = n2[r[j].y][r[j].x];
			cv::Mat nr2_h(4,1,CV_64F);
			
			nr2_h.at<double>(0) = nr2.at<double>(0);
			nr2_h.at<double>(1) = nr2.at<double>(1);
			nr2_h.at<double>(2) = nr2.at<double>(2);
			nr2_h.at<double>(3) = 1.0;

			cv::Mat nrw_h = Mw2.inv()*nr2_h;
			nrw_h.at<double>(0) -= cam2.pos.at<double>(0);
			nrw_h.at<double>(1) -= cam2.pos.at<double>(1);
			nrw_h.at<double>(2) -= cam2.pos.at<double>(2);
			nrw_h = nrw_h/calc_norm(nrw_h,3);

			/*std::cout << nrw_h << std::endl
				<< nlw_h << std::endl;
			cvWaitKey();
			*/

			if (j > i) {
				std::cout << "error: " << j << " " << i << std::endl;
				exit(1);
			}
			cost[j][i] = calc_norm(nrw_h-nlw_h,3) / 2.0;
		}
	}
	return cost;
}
//**************************************************************************************+

std::vector<std::vector<double> >  calc_matching_cost(std::vector<std::vector<cv::Point> >  lines,IplImage* im1, IplImage* im2) {
	
	std::vector<cv::Point> l = lines[0];
	std::vector<cv::Point> r = lines[1];

	//float fscale = (float)n1[0].size()/640.0;

	

	std::vector<std::vector<double> >  cost(r.size(),std::vector<double>(l.size(),1));

	int l_top;
	if (l.size() > r.size()) 
		l_top = r.size();
	else 
		l_top = l.size();
	for (int i = 0; i < l_top; i++) {
		cv::Mat n1(3,1,CV_64F);
		n1.at<double>(0) = CV_IMAGE_ELEM(im1,uchar,l[i].y,l[i].x*3);
		n1.at<double>(1) = CV_IMAGE_ELEM(im1,uchar,l[i].y,l[i].x*3+1);
		n1.at<double>(2) = CV_IMAGE_ELEM(im1,uchar,l[i].y,l[i].x*3+2);
		
		int top;
		if (r.size()-1 < i)
			top = r.size()-1;
		else
			top = i;
		
		for (int j = 0; j <= top; j++) {
			
			cv::Mat n2(3,1,CV_64F);
			n2.at<double>(0) = CV_IMAGE_ELEM(im2,uchar,r[j].y,r[j].x*3);
			n2.at<double>(1) = CV_IMAGE_ELEM(im2,uchar,r[j].y,r[j].x*3+1);
			n2.at<double>(2) = CV_IMAGE_ELEM(im2,uchar,r[j].y,r[j].x*3+2);
			

			
			cost[j][i] = calc_norm(n1-n2,3) / 255;
		}
	}
	return cost;
}


double Clo (int l,int r) {

	double c1 = INF;
	double c2 = INF;
	double c3 = INF;
	double cost;
	
	int min_idx;

	
	//std::cout << "Clo: " << l << " " << r << std::endl;
	//cvWaitKey();

	if (clo[r][l] < INF) {
		cost = clo[r][l];
	}
	else {
		if (r > 0) {
			c1 = Clo(l,r-1) + ALPHA;
			c2 = Clm(l,r-1) + BETA;
			c3 = Crm(l,r-1) + BETA;

			min_idx = 1;
			cost = c1;
			if (cost > c2) {cost = c2; min_idx = 2;}
			if (cost > c3) {cost = c3; min_idx = 3;}

			
			if (min_idx==1) {
				bl_lo[r][l] = 1;
			}
			else if (min_idx == 2) {
				bl_lo[r][l] = 2;
			}
			else 
				bl_lo[r][l] = 3;
		}
		else {
			cost = INF;
			bl_lo[r][l]=-1;
		}
		clo[r][l] = cost;
	}
	return cost;
}


double Clm (int l,int r) {

	double c1 = INF;
	double c2 = INF;
	double c3 = INF;
	double c4 = INF;
	double cost;

	//std::cout << "Clm: " << l << " " << r << std::endl;
	//cvWaitKey();
	
	int min_idx;

	if (clm[r][l] < INF) {
		cost = clm[r][l];
	}
	else {
		if (r > 0) {
			c1 = Clo(l,r-1) + BETA_P;
			c2 = Clm(l,r-1) + GAMMA;
			c3 = Crm(l,r-1);
			c4 = Cro(l,r-1) + BETA_P;
			
			min_idx = 1;
			cost = c1;
			if (cost > c2) {cost = c2; min_idx = 2;}
			if (cost > c3) {cost = c3; min_idx = 3;}
			if (cost > c4) {cost = c4; min_idx = 4;}
			
			cost = costs[r][l] + cost;

			if (min_idx==1) {
				bl_lm[r][l] = 4;
			}
			else if (min_idx == 2) {
				bl_lm[r][l] = 5;
			}
			else if (min_idx == 3) {
				bl_lm[r][l] = 6;
			}
			else {
				bl_lm[r][l] = 7;
			}
		}
		else {
			cost = INF;
			bl_lm[r][l]=-1;
		}
		clm[r][l] = cost;
	}
	return cost;
}


double Cro (int l,int r) {

	double c1 = INF;
	double c2 = INF;
	double c3 = INF;
	double cost;
	
	int min_idx;

	//std::cout << "Cro: " << l << " " << r << std::endl;
	//cvWaitKey();
	
	if (cro[r][l] < INF) {
		cost = cro[r][l];
	}
	else {
		if ((l > 0) && (l > r)){
			c1 = Cro(l-1,r) + ALPHA;
			c2 = Crm(l-1,r) + BETA;
			c3 = Clm(l-1,r) + BETA;

			min_idx = 1;
			cost = c1;
			if (cost > c2) {cost = c2; min_idx = 2;}
			if (cost > c3) {cost = c3; min_idx = 3;}

			
			if (min_idx==1) {
				bl_ro[r][l] = 8;
			}
			else if (min_idx == 2) {
				bl_ro[r][l] = 9;
			}
			else 
				bl_ro[r][l] = 10;
		}
		else {
			cost = INF;
			bl_ro[r][l]=-1;
		}
		cro[r][l] = cost;
	}
	return cost;
}

double Crm (int l,int r) {

	double c1 = INF;
	double c2 = INF;
	double c3 = INF;
	double c4 = INF;
	double cost;
	
	int min_idx;

	//std::cout << "Crm: " << l << " " << r << std::endl;
	//cvWaitKey();

	if (crm[r][l] < INF) {
		cost = crm[r][l];
	}
	else {
		if ((l > 0) && (l > r)){
			c1 = Cro(l-1,r) + BETA_P;
			c2 = Crm(l-1,r) + GAMMA;
			c3 = Clm(l-1,r);
			c4 = Clo(l-1,r) + BETA_P;
			
			min_idx = 1;
			cost = c1;
			if (cost > c2) {cost = c2; min_idx = 2;}
			if (cost > c3) {cost = c3; min_idx = 3;}
			if (cost > c4) {cost = c4; min_idx = 4;}

			cost = costs[r][l] + cost;

			if (min_idx==1) {
				bl_rm[r][l] =11;
			}
			else if (min_idx == 2) {
				bl_rm[r][l] = 12;
			}
			else if (min_idx == 3) {
				bl_rm[r][l] = 13;
			}
			else {
				bl_rm[r][l] = 14;
			}
		}
		else {
			cost = INF;
			bl_rm[r][l]=-1;
		}
		crm[r][l] = cost;
	}
	return cost;
}



std::vector<std::vector<cv::Point> >  find_best_path(std::vector<std::vector<double> >  matching_costs,std::vector<std::vector<cv::Point> >  lines,IplImage*left_im, IplImage* right_im, int w, int h) {


	std::vector<std::vector<cv::Point> >  pixel_matches = std::vector<std::vector<cv::Point> > (2);
	costs = matching_costs;

	float scale_f = (float)w/640.0;
	
	cvNamedWindow("min path");
	
	IplImage* matching_im = cvCreateImage(cvSize(costs[0].size(),costs.size()),IPL_DEPTH_8U,3);
	IplImage* big_im = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
	big_im->origin = 1;
	IplImage* aux_im = cvCreateImage(cvSize(costs[0].size(),costs.size()),IPL_DEPTH_8U,1);
	
	for (int i = 0; i < costs.size(); i++) {
		for (int j = 0; j < costs[0].size(); j++) {
			CV_IMAGE_ELEM(aux_im, uchar, i, j ) = (unsigned char)((costs[i][j]*255));
		}
	}
	
	cvCvtColor(aux_im, matching_im, CV_GRAY2RGB);
	
	/*
	cvResize(matching_im, big_im,0);
	cvShowImage("min path", big_im);

	cvWaitKey();
	*/
	
	//init cumulative cost matrices
	clo = std::vector<std::vector<double> > (costs.size(),std::vector<double>(costs[0].size(),INF));
	clm = std::vector<std::vector<double> > (costs.size(),std::vector<double>(costs[0].size(),INF));
	cro = std::vector<std::vector<double> > (costs.size(),std::vector<double>(costs[0].size(),INF));
	crm = std::vector<std::vector<double> > (costs.size(),std::vector<double>(costs[0].size(),INF));

	//init backlinks
	bl_lo = std::vector<std::vector<double> > (costs.size(),std::vector<double>(costs[0].size(),0));
	bl_lm = std::vector<std::vector<double> > (costs.size(),std::vector<double>(costs[0].size(),0));
	bl_ro = std::vector<std::vector<double> > (costs.size(),std::vector<double>(costs[0].size(),0));
	bl_rm = std::vector<std::vector<double> > (costs.size(),std::vector<double>(costs[0].size(),0));

	cro[0][0] = 0;

	//find best path
	int l = costs[0].size()-1;
	int r = costs.size()-1;
	
	//std::cout << "l:" <<l << std::endl
	//	<< "r: " << r << std::endl;

		

	double min_path = Clo (l,r); 
	

	


	CvScalar color = cvScalar(255,0,0);
	

	int link = bl_lo[r][l];
	while (l > 0 || r > 0) {
		//std::cout << "["<<l <<","<<r<<"]"<<std::endl;
		
		CV_IMAGE_ELEM(matching_im, uchar,r,l*3) = color.val[0];
		CV_IMAGE_ELEM(matching_im, uchar,r,l*3+1) = color.val[1];
		CV_IMAGE_ELEM(matching_im, uchar,r,l*3+2) = color.val[2];
		
		
		switch (link) {
		case 1:
			r = r -1; link = bl_lo[r][l]; color = cvScalar(255,0,0);break;
		case 2:
			r = r -1; link = bl_lm[r][l]; color = cvScalar(0,0,255);/*pixel_matches[0].push_back(lines[0][l]);pixel_matches[1].push_back(lines[1][r]);*/break;
		case 3:
			r = r -1; link = bl_rm[r][l]; color = cvScalar(255,0,255);/*pixel_matches[0].push_back(lines[0][l]);pixel_matches[1].push_back(lines[1][r]);*/break;
		case 4:
			r = r -1; link = bl_lo[r][l]; color = cvScalar(255,0,0);break;
		case 5:
			r = r -1; link = bl_lm[r][l]; color = cvScalar(0,0,255);/*pixel_matches[0].push_back(lines[0][l]);pixel_matches[1].push_back(lines[1][r]);*/break;
		case 6:
			r = r -1; link = bl_rm[r][l]; color = cvScalar(255,0,255);pixel_matches[0].push_back(lines[0][l]);pixel_matches[1].push_back(lines[1][r]);break;
		case 7:
			r = r -1; link = bl_ro[r][l]; color = cvScalar(0,255,0);break;
		case 8:
			l = l -1; link = bl_ro[r][l]; color = cvScalar(0,255,0);break;
		case 9:
			l = l -1; link = bl_rm[r][l]; color = cvScalar(255,0,255);/*pixel_matches[0].push_back(lines[0][l]);pixel_matches[1].push_back(lines[1][r]);*/break;
		case 10:
			l = l -1; link = bl_lm[r][l]; color = cvScalar(0,0,255);/*pixel_matches[0].push_back(lines[0][l]);pixel_matches[1].push_back(lines[1][r]);*/break;
		case 11:
			l = l -1; link = bl_ro[r][l]; color = cvScalar(0,255,0);break;
		case 12:
			l = l -1; link = bl_rm[r][l]; color = cvScalar(255,0,255);/*pixel_matches[0].push_back(lines[0][l]);pixel_matches[1].push_back(lines[1][r]);*/break;
		case 13:
			l = l -1; link = bl_lm[r][l]; color = cvScalar(0,0,255);pixel_matches[0].push_back(lines[0][l]);pixel_matches[1].push_back(lines[1][r]);break;
		case 14:
			l = l -1; link = bl_lo[r][l]; color = cvScalar(255,0,0);break;
		}
		/*CV_IMAGE_ELEM(left_im, uchar,lines[0][l].y,lines[0][l].x*3) = color.val[2];
		CV_IMAGE_ELEM(left_im, uchar,lines[0][l].y,lines[0][l].x*3+1) = color.val[1];
		CV_IMAGE_ELEM(left_im, uchar,lines[0][l].y,lines[0][l].x*3+2) = color.val[0];
		
		CV_IMAGE_ELEM(right_im, uchar,lines[1][r].y,lines[1][r].x*3) = color.val[2];
		CV_IMAGE_ELEM(right_im, uchar,lines[1][r].y,lines[1][r].x*3+1) = color.val[1];
		CV_IMAGE_ELEM(right_im, uchar,lines[1][r].y,lines[1][r].x*3+2) = color.val[0];
		*/
		int temp = color.val[0];
		color.val[0] = color.val[2];
		color.val[2] = temp;
		cvCircle(left_im,lines[0][l]*(1/scale_f),1,color,2); 
		cvCircle(right_im,lines[1][r]*(1/scale_f),1,color,2); 
	}
	/*
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.4, 0.4);
	for (int i = 0; i < pixel_matches[0].size(); i++) {
		std::stringstream ss;
		ss<<i;
		std::string match = ss.str();
		cvPutText(left_im, match.c_str(),pixel_matches[0][i]*(1/scale_f),&font,cvScalar(255,255,255));
		cvPutText(right_im, match.c_str(),pixel_matches[1][i]*(1/scale_f),&font,cvScalar(255,255,255));
		//cvCircle(left_im,pixel_matches[0][i],1,cvScalar(255,255,0),2);
		//cvCircle(right_im,pixel_matches[1][i],1,cvScalar(255,255,0),2);
	}*/

	
	CV_IMAGE_ELEM(matching_im, uchar,r,l*3) = color.val[0];
	CV_IMAGE_ELEM(matching_im, uchar,r,l*3+1) = color.val[1];
	CV_IMAGE_ELEM(matching_im, uchar,r,l*3+2) = color.val[2];
		

	//cvResize(matching_im, big_im,0);
	//cvShowImage("min path", big_im);
	//cvWaitKey();
	
	return pixel_matches;
}

