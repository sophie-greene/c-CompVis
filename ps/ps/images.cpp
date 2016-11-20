
#include "images.h"
#include "geometric_calibration.h"




using namespace std;
//using namespace cv;


/***************************************************************************
Read images from files. Sometimes there are no images for every light source, 
for example when the image is blurry or to dark, and is discarded. 
Input:
	path: path where the files are
	available_lights: vector that indicates the index of the light corresponding to each image
	image_scale_f: we want to resize the images when readed according to this scaling factor
Output:
	vector containing images
******************************************************************************/

vector<IplImage*> read_images(string path,int cam_num,vector<int> &available_lights, float image_scale_f) {

	vector<IplImage*> image_list;

	
	string cam = "cam";
	string light = "light";
	string ext = ".bmp";
	stringstream ss;
	ss << cam_num;
	string cam_idx = ss.str();
	string filename; 
	for (int i = 0; i < CAM_NUM; i++) {
		stringstream ss;
		ss << i;
		string light_idx = ss.str();
		filename = path + cam + cam_idx + light + light_idx + ext;
		//cout << filename << endl;
		IplImage* temp_image = cvLoadImage(filename.c_str(),1);
		if (temp_image!=NULL) {
			available_lights.push_back(i);
			IplImage* temp_scaled_image = cvCreateImage(cvSize(int(temp_image->width*image_scale_f),int(temp_image->height*image_scale_f)),IPL_DEPTH_8U,3);
			cvResize(temp_image, temp_scaled_image);
			image_list.push_back(temp_scaled_image);
		}
		else {
			cout << "no esta la imagen " << filename.c_str() << endl;
		}
	}

	return image_list;
}

/*********************************************************************
Read mask file corresponding to the a camera number
Input:
	path: path where to mask image is
	cam_num: number of the camera
	image_scale_f: scaling factor to reduce the image
**********************************************************************/

IplImage* read_mask(string path,int cam_num, float image_scale_f){
	
	string cam = "cam";
	string mask = "mask.jpg";
	
	stringstream ss;
	ss << cam_num;
	string cam_idx = ss.str();
	string filename; 
	filename = path + cam + cam_idx + mask;
	IplImage* mask_image = cvLoadImage(filename.c_str(),0);
	IplImage* mask_scaled_image = cvCreateImage(cvSize(int(mask_image->width*image_scale_f),int(mask_image->height*image_scale_f)),IPL_DEPTH_8U,1);
	cvResize(mask_image, mask_scaled_image);
	return mask_scaled_image;
} 
