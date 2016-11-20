
#include "groundTruth.h"
#define FRAME_OFFSET 0
#define INTERSECTION_THRESH 1
using namespace std;

vector<vector<cv::Point> > read_intrinsic_parameters(string file_name) {

    std::ifstream read(file_name.c_str());
    string line;
    int val[16];//,val1,val2,val3,val4,val5,val7,val8,val;
    vector<vector<cv::Point> >  pixels;

    char a;

    //while ( !read.eof()) {
    //read>>line;
    //read>>line;
    int max_x = 0;
    int max_y = 0;
    while(read>>line>>line>>line>>a>>val[0]>>
          a>>val[1]>>a>>val[2]>>a>>val[3]>>a>>
          val[4]>>a>>val[5]>>a>>val[6]>>a>>val[7]>>
          a>>val[8]>>a>>val[9]>>a>>val[10]>>a>>val[11]>>a>>val[12]>>
          a>>val[13]>>a>>val[14]>>a>>val[15]>>a) {
        vector<cv::Point> aux_v;
        cv::Point aux_p;
        for (int i = 0; i < 16; i+=2) {
            int x = val[i];
            int y = val[i+1];
            if (x > max_x) max_x = x;
            if (y > max_y) max_y = y;
            //if (x == -1) continue;
            aux_p.x = x;
            aux_p.y = y;
            aux_v.push_back(aux_p);
            //		cout << " " << aux_p;
        }
        //cout << endl;
        pixels.push_back(aux_v);
    }
    //cout << max_x << " " << max_y << endl;
    return pixels;
}


void build_ground_truth_cloud(pcl::visualization::PCLVisualizer &viewer, vector<cam_params_t> params, vector<vector<cv::Point> > laser_pixels) {



    //cout << "Pl: " << Pl << endl;
    //cout << "Pr: " << Pr << endl;
    /*vector<IplImage*> images(8);
	vector<IplImage*> small_images(8);
	for (int i = 0; i < 8; i++)
		small_images[i] = cvCreateImage(cvSize(176,144),IPL_DEPTH_8U,3);	
	images[0] = cvLoadImage("cam0.bmp");
	images[1] = cvLoadImage("cam1.bmp");
	images[2] = cvLoadImage("cam2.bmp");
	images[3] = cvLoadImage("cam3.bmp");
	images[4] = cvLoadImage("cam4.bmp");
	images[5] = cvLoadImage("cam5.bmp");
	images[6] = cvLoadImage("cam6.bmp");
	images[7] = cvLoadImage("cam7.bmp");

	
	cvNamedWindow("1");
	cvNamedWindow("2");
	cvNamedWindow("3");
	cvNamedWindow("4");
	cvNamedWindow("5");
	cvNamedWindow("6");
	cvNamedWindow("7");
	cvNamedWindow("8");
	*/

    //int cam_trad[] = {2,3,4,5,6,7,1,0}; //manual

    vector<int> cam_counter(8,0);
    int cam_trad[] = {0,1,2,3,4,5,6,7};


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    float scalef_x = 1;//176.0/640.0;
    float scalef_y = 1;// 144.0/480.0;
    int cnt = 0;
    int total = 0;
    for (int i = FRAME_OFFSET; i < laser_pixels.size(); i++) {
        //cout << i << "/" << laser_pixels.size()<< endl;
	/*	cvResize(images[0],small_images[0]);
		cvResize(images[1],small_images[1]);
		cvResize(images[2],small_images[2]);
		cvResize(images[3],small_images[3]);
		cvResize(images[4],small_images[4]);
		cvResize(images[5],small_images[5]);
		cvResize(images[6],small_images[6]);
		cvResize(images[7],small_images[7]);
		*/
        for (int j = 0; j < 8; j++) {
            int x1 = laser_pixels[i][j].x;
            int y1 = laser_pixels[i][j].y;
            if (x1==-1) continue;
            double min_dist = 999999999999999, min_dist2;
            int min_x2, min_y2, min_k;
            pcl::PointXYZ p3D;
            for (int w = -FRAME_OFFSET; w <= FRAME_OFFSET; w++) {
                if (i+w >= laser_pixels.size()) break;
                for (int k = 0; k < 8; k++) {
                    if (j==k) continue;
                    int x2 = laser_pixels[i+w][k].x;
                    int y2 = laser_pixels[i+w][k].y;
                    if (x2==-1) continue;

                    cv::Mat p1(3,1,CV_64F);
                    cv::Mat p2(3,1,CV_64F);
                    cv::Mat Pl = compound_cam_transformation(params[cam_trad[j]].cam,params[cam_trad[j]].rot,params[cam_trad[j]].trans);
                    cv::Mat Pr = compound_cam_transformation(params[cam_trad[k]].cam,params[cam_trad[k]].rot,params[cam_trad[k]].trans);
                    p1.at<double>(0) = (double)(x1/scalef_x);
                    p1.at<double>(1) = (double)(y1/scalef_y);
                    p1.at<double>(2) = 1.0;

                    p2.at<double>(0) = (double)(x2/scalef_x);
                    p2.at<double>(1) = (double)(y2/scalef_y);
                    p2.at<double>(2) = 1.0;

                    cv::Mat aux = Pl.t()*(Pl*Pl.t()).inv()*p1;

                    aux /= aux.at<double>(3);
                    cv::Mat v1_h(3,1,CV_64F);

                    v1_h.at<double>(0) = aux.at<double>(0) - params[cam_trad[j]].pos.at<double>(0);
                    v1_h.at<double>(1) = aux.at<double>(1) - params[cam_trad[j]].pos.at<double>(1);
                    v1_h.at<double>(2) = aux.at<double>(2) - params[cam_trad[j]].pos.at<double>(2);



                    aux = Pr.t()*(Pr*Pr.t()).inv()*p2;
                    aux /= aux.at<double>(3);
                    cv::Mat v2_h(3,1,CV_64F);

                    v2_h.at<double>(0) = aux.at<double>(0) - params[cam_trad[k]].pos.at<double>(0);
                    v2_h.at<double>(1) = aux.at<double>(1) - params[cam_trad[k]].pos.at<double>(1);
                    v2_h.at<double>(2) = aux.at<double>(2) - params[cam_trad[k]].pos.at<double>(2);

                    Eigen::VectorXf line1_coeff(6);
                    Eigen::VectorXf line2_coeff(6);

                    line1_coeff[0] = params[cam_trad[j]].pos.at<double>(0);
                    line1_coeff[1] = params[cam_trad[j]].pos.at<double>(1);
                    line1_coeff[2] = params[cam_trad[j]].pos.at<double>(2);

                    line1_coeff[3] = v1_h.at<double>(0);
                    line1_coeff[4] = v1_h.at<double>(1);
                    line1_coeff[5] = v1_h.at<double>(2);

                    line2_coeff[0] = params[cam_trad[k]].pos.at<double>(0);
                    line2_coeff[1] = params[cam_trad[k]].pos.at<double>(1);
                    line2_coeff[2] = params[cam_trad[k]].pos.at<double>(2);

                    line2_coeff[3] = v2_h.at<double>(0);
                    line2_coeff[4] = v2_h.at<double>(1);
                    line2_coeff[5] = v2_h.at<double>(2);

                    Eigen::Vector4f intersection_point;
                    //if (pcl::lineWithLineIntersection(line1_coeff,line2_coeff,intersection_point,INTERSECTION_THRESH)) {
                    //cout << j << " / "<<x1 << " "<<y1<<" "<<k<<" / "<<x2<<" "<<y2<<endl;

                    pcl::lineWithLineIntersection(line1_coeff,line2_coeff,intersection_point,10);
                    //cout << intersection_point << endl;
                    p3D.x = intersection_point[0];
                    p3D.y = intersection_point[1];
                    p3D.z = intersection_point[2];
                    cloud->points.push_back(p3D);
                    continue;

                    for (int w2 = -FRAME_OFFSET; w2 <= FRAME_OFFSET; w2++) {
                        if (i+w2 >= laser_pixels.size()) break;
                        for (int k2 = 0; k2 < 8; k2++) {
                            if ((j==k2) || (k==k2))  continue;
                            int x3 = laser_pixels[i+w2][k2].x;
                            int y3 = laser_pixels[i+w2][k2].y;
                            if (x3==-1) continue;


                            cv::Mat p3(3,1,CV_64F);
                            //++total;

                            cv::Mat Pr2 = compound_cam_transformation(params[k2].cam,params[k2].rot,params[k2].trans);

                            /*cout << "---------------------"<<endl;
						cout << pixel_matches[0][i] << endl;
						cout << pixel_matches[1][i] << endl;
						cout << "---------------------"<<endl;
						*/



                            p3.at<double>(0) = (double)(x3/scalef_x);
                            p3.at<double>(1) = (double)(y3/scalef_y);
                            p3.at<double>(2) = 1.0;





                            aux = Pr2.t()*(Pr2*Pr2.t()).inv()*p3;
                            aux /= aux.at<double>(3);
                            cv::Mat v3_h(3,1,CV_64F);

                            v3_h.at<double>(0) = aux.at<double>(0) - params[k2].pos.at<double>(0);
                            v3_h.at<double>(1) = aux.at<double>(1) - params[k2].pos.at<double>(1);
                            v3_h.at<double>(2) = aux.at<double>(2) - params[k2].pos.at<double>(2);


                            Eigen::VectorXf line3_coeff(6);




                            line3_coeff[0] = params[k2].pos.at<double>(0);
                            line3_coeff[1] = params[k2].pos.at<double>(1);
                            line3_coeff[2] = params[k2].pos.at<double>(2);

                            line3_coeff[3] = v3_h.at<double>(0);
                            line3_coeff[4] = v3_h.at<double>(1);
                            line3_coeff[5] = v3_h.at<double>(2);



                            Eigen::Vector4f intersection_point2;

                            //if (pcl::lineWithLineIntersection(line1_coeff,line2_coeff,intersection_point,0.5)) {

                            if (pcl::lineWithLineIntersection(line1_coeff,line3_coeff,intersection_point2,INTERSECTION_THRESH)) {
                                float point_dist = sqrt(double(
                                        ((intersection_point[0]-intersection_point2[0])*(intersection_point[0]-intersection_point2[0]))
                                        +((intersection_point[1]-intersection_point2[1])*(intersection_point[1]-intersection_point2[1]))
                                        +((intersection_point[2]-intersection_point2[2])*(intersection_point[2]-intersection_point2[2]))
                                        ));
                                if (point_dist<INTERSECTION_THRESH) {
                                    ++cam_counter[j];
                                    ++cam_counter[k];
                                    ++cam_counter[k2];
                                    p3D.x = intersection_point[0];
                                    p3D.y = intersection_point[1];
                                    p3D.z = intersection_point[2];
                                    cloud->points.push_back(p3D);
                                }
                            }


                            continue;
                            cv::Mat aux_p(3,1,CV_64F);
                            aux_p.at<double>(0) = intersection_point[0];
                            aux_p.at<double>(1) = intersection_point[1];
                            aux_p.at<double>(2) = intersection_point[2];
                            cv::Mat n_v(3,1,CV_64F);
                            n_v.at<double>(0) = v1_h.at<double>(0);
                            n_v.at<double>(1) = v1_h.at<double>(1);
                            n_v.at<double>(2) = v1_h.at<double>(2);
                            n_v /= calc_norm(n_v,3);
                            float dist = calc_norm((params[j].pos - aux_p) -((params[j].pos - aux_p).dot(n_v))*n_v,3);
                            n_v.at<double>(0) = v2_h.at<double>(0);
                            n_v.at<double>(1) = v2_h.at<double>(1);
                            n_v.at<double>(2) = v2_h.at<double>(2);
                            n_v /= calc_norm(n_v,3);

                            float dist2 = calc_norm((params[k].pos - aux_p) -((params[k].pos - aux_p).dot(n_v))*n_v,3);
                            //	cout << dist<<endl;

                            if (dist < min_dist) {
                                min_k = k;
                                min_x2 = x2;
                                min_y2 = y2;
                                min_dist = dist;
                                min_dist2 = dist2;
                                p3D.x = intersection_point[0];
                                p3D.y = intersection_point[1];
                                p3D.z = intersection_point[2];
                            }






                            //}
                            //else {
                            //cout << "No intersection" << endl;
                            //	++cnt;
                            //}

                        }
                    }
                    //}
                }
            }
            /*	if (j == min_k) {
				cout << "Error " <<endl;
				exit(0);
			}
			//cvDrawCircle(small_images[j],cvPoint(x1,y1),2,cvScalar(255,0,0,0),2);
			//cvDrawCircle(small_images[min_k],cvPoint(min_x2,min_y2),2,cvScalar(255,0,0,0),2);
			if ((min_dist < 0.05) && (min_dist2 < 0.05)) {
				cout << j << " [" << x1<<" "<<y1<<"] "<<min_k<<" [" << min_x2<<" "<<min_y2<<"] "<<"dist " << min_dist << " " << min_dist2 << endl;

				cloud->points.push_back(p3D);
			}
			*/
        }
	/*	cvShowImage("1",small_images[0]);
		cvShowImage("2",small_images[1]);
		cvShowImage("3",small_images[2]);
		cvShowImage("4",small_images[3]);
		cvShowImage("5",small_images[4]);
		cvShowImage("6",small_images[5]);
		cvShowImage("7",small_images[6]);
		cvShowImage("8",small_images[7]);
		*/
        //cvWaitKey();
    }
    //cout << cnt << "/" << total<<endl;
    for (int i = 0; i < 8; i++)
        cout <<cam_counter[i] << endl;
    viewer.addPointCloud(cloud,"groundtruth");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"groundtruth");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"groundtruth");

}

void test(vector<vector<cv::Point> > laser_pixels) {


    vector<IplImage*> images(8);
    vector<IplImage*> small_images(8);

    vector<vector<cv::Point> > pixels(8);
    for (int i = 0; i < 8; i++)
        small_images[i] = cvCreateImage(cvSize(176,144),IPL_DEPTH_8U,3);
    images[0] = cvLoadImage("cam0.bmp");
    images[1] = cvLoadImage("cam1.bmp");
    images[2] = cvLoadImage("cam2.bmp");
    images[3] = cvLoadImage("cam3.bmp");
    images[4] = cvLoadImage("cam4.bmp");
    images[5] = cvLoadImage("cam5.bmp");
    images[6] = cvLoadImage("cam6.bmp");
    images[7] = cvLoadImage("cam7.bmp");


    cvNamedWindow("1");
    cvNamedWindow("2");
    cvNamedWindow("3");
    cvNamedWindow("4");
    cvNamedWindow("5");
    cvNamedWindow("6");
    cvNamedWindow("7");
    cvNamedWindow("8");
    float scalef_x = 176.0/640.0;
    float scalef_y = 144.0/480.0;
    for (int i = 5; i < laser_pixels.size(); i++) {
        cvResize(images[0],small_images[0]);
        cvResize(images[1],small_images[1]);
        cvResize(images[2],small_images[2]);
        cvResize(images[3],small_images[3]);
        cvResize(images[4],small_images[4]);
        cvResize(images[5],small_images[5]);
        cvResize(images[6],small_images[6]);
        cvResize(images[7],small_images[7]);
        //cout << laser_pixels[i] << endl;

        for (int j = 0; j < 8; j++) {
            int x1 = laser_pixels[i][j].x;
            int y1 = laser_pixels[i][j].y;
            if (x1==-1) continue;
            //x1 = x1/scalef_x;
            //y1 = y1/scalef_y;
            pixels[j].push_back(cvPoint(x1,y1));




            /*for (int k = 0; k < 8; k++) {
					if (j==k) continue;
					int x2 = laser_pixels[i][k].x;
					int y2 = laser_pixels[i][k].y;
					if (x2==-1) continue;
					//x2 = x2/scalef_x;
					//y2 = y2/scalef_y;
					
					
				}
				*/

        }
        vector<CvScalar> colors(5);

        colors[0] = cvScalar(255,0,0);
        colors[1] = cvScalar(0,0,255);
        colors[2] = cvScalar(255,255,0);
        colors[3] = cvScalar(0,255,255);
        colors[4] = cvScalar(255,0,255);

        for (int j = 0; j < 8; j++) {
            for (int w = pixels[j].size()-1;w > (pixels[j].size()-6); w--) {

                if (w < 0) continue;
                //cout << "["<<j<<"] " <<pixels[j][w]<<" ";
                cvDrawCircle(small_images[j],pixels[j][w],2,colors[(pixels[j].size()-1)-w],2);
            }
            //	cout << endl;

        }
        cvShowImage("1",small_images[0]);
        cvShowImage("2",small_images[1]);
        cvShowImage("3",small_images[2]);
        cvShowImage("4",small_images[3]);
        cvShowImage("5",small_images[4]);
        cvShowImage("6",small_images[5]);
        cvShowImage("7",small_images[6]);
        cvShowImage("8",small_images[7]);
        cvWaitKey();
    }
}
