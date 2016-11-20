#include "normal_map.h"
#include "utils.h"

using namespace std;
//using namespace cv;


/*********************************************************************************+
Calculates the normal map using: I = albedo*N*L where N is the normal of the surface and
L is the light direction. Overdetermined system is constructed:
	I1 = albedo*(Nx*L1x + Ny*L1y + NzL1z)
	I2 = albedo*(Nx*L2x + Ny*L2y + NzL2z)
	I3 = albedo*(Nx*L3x + Ny*L3y + NzL3z)
	.
	.
	.
	In = albedo*(Nx*Lnx + Ny*Lny + NzLnz)
We need more than 3 ecuations (light sources) to solve the 3 unknowns (Nx,Ny,Nz). System is solved by
linear least squares.

Input: 
	images: vector containing the images corresponding to different light sources
	mask_iamge: we do not compute normals for pixels outside the mask
	l: light directions
	m: compound rotation and translation matrix
	trans. translation vector for the cam
Output:
	normal map, each normal is a cv::Mat containing x,y,z coordinates of the normal
***********************************************************************************************/


vector<vector<cv::Mat> > calc_normal_map(vector<IplImage*> images, IplImage* mask_image, cv::Mat l, cv::Mat m, cv::Mat trans) {
	
	
	//cout << images[1]->height << " " << images[1]->width << endl;
        vector<vector<cv::Mat> > normals(images[1]->height, vector<cv::Mat>(images[1]->width));
	
	/*
	cvNamedWindow("normal source image");
	cvNamedWindow("normal mask");

	cvShowImage("normal source image",images[0]);
	cvShowImage("normal mask",mask_image);
	*/
	
	//cout << l << endl;
	
	//for every pixel of the image
	for (int i = 0; i < images[1]->height; i++) {
		for (int j = 0; j < images[1]->width; j++) {
			normals[i][j] = cv::Mat::zeros(3,1,CV_64F);
			cv::Mat nw(4,1,CV_64F);
			//normal pointing up in world coordinates(from the floor pointing up)
			nw.at<double>(0) = 0;
			nw.at<double>(1) = 0;
			nw.at<double>(2) = 1.0;
			nw.at<double>(3) = 1.0;
			
			//transform the normal to cam coordinates
			cv::Mat n = m*nw;

			//represent the normal with respect to the world origin as seen by the camera
			normals[i][j].at<double>(0) = n.at<double>(0)-trans.at<double>(0);
			normals[i][j].at<double>(1) = n.at<double>(1)-trans.at<double>(1);
			normals[i][j].at<double>(2) = n.at<double>(2)-trans.at<double>(2);
			normals[i][j] = normals[i][j]/calc_norm(normals[i][j],3);
			

			cv::Mat default_normal = normals[i][j];
			
			if (CV_IMAGE_ELEM(mask_image, uchar, i,j) > 100 ) {		//if we are inside the mask
				
				//fill intensities for each pixel 
				cv::Mat intensities(images.size(),1,CV_64F);
				double *intensities_data = (double*) (intensities.data);
				
				for (int k = 0; k < images.size(); k++) {	
					intensities_data[k] = (double)(CV_IMAGE_ELEM(images[k],uchar,i,j))/255.0;
				}

				//use the normal of the intensities vector to determine is the pixel is feasible to
				//compute normal in it. Not to bright (specularities) not to dark, because in these cases
				//normal computation will be prone to error.
				double aux = calc_norm(intensities,images.size());
				if (aux >= 0.1 && aux <= 0.8) {
					
					//solving the least squares by normal equations
					cv::Mat tb= (l.t()*l).inv()*l.t()*intensities;
					tb = tb/calc_norm(tb,3);
					cv::Mat aux(4,1,CV_64F);
					aux.at<double>(0) = tb.at<double>(0);
					aux.at<double>(1) = tb.at<double>(1);
					aux.at<double>(2) = tb.at<double>(2);
					aux.at<double>(3) = 1;
					//aux = m*aux;
					
					//this is a fix to avoid surface generation do strange things. When normal is very flat (z is very small)
					//depth map generation behabes poorly. That is why is the calculated normal is very flat, we change it by the default normal

					if (fabs(aux.at<double>(2)) < 0.1) {
						aux.at<double>(0) = default_normal.at<double>(0);
						aux.at<double>(1) = default_normal.at<double>(1);
						aux.at<double>(2) = default_normal.at<double>(2);
					}
					
					normals[i][j].at<double>(0) = aux.at<double>(0);
					normals[i][j].at<double>(1) = aux.at<double>(1);
					normals[i][j].at<double>(2) = aux.at<double>(2);
				}
				//else {
				//	CV_IMAGE_ELEM(mask_image, uchar, i,j) = 0;
				//}
			}
			
		}
	}
	

	//show normals
	/*
	IplImage* normal_im = cvCreateImage(cvSize(images[1]->width,images[1]->height),IPL_DEPTH_8U,3);
	
	for (int i = 0; i < images[1]->height; i++) {
		for (int j = 0; j < images[1]->width; j++) {
			CV_IMAGE_ELEM(normal_im, uchar, i, (j * 3)) = (unsigned char)((normals[i][j].at<double>(2)+1)/2*255);
			CV_IMAGE_ELEM(normal_im, uchar, i, (j * 3)+1) = (unsigned char)((normals[i][j].at<double>(1)+1)/2*255);
			CV_IMAGE_ELEM(normal_im, uchar, i, (j * 3)+2) = (unsigned char)((normals[i][j].at<double>(0)+1)/2*255);
			
		}
	}
	cvNamedWindow("normals image");
	cvShowImage("normals image",normal_im);
	cvWaitKey();
	*/
	return normals;
}
