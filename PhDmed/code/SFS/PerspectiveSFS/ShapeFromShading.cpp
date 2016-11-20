/////////////////////////////////////////////////////////////
// Name: ShapeFromShading.cpp
//
// The code is provided "as-is" with no express or implied warranty for accuracy or compatibility.
//
// If using the code, please cite the following paper:
// 
// M. Visentini-Scarzanella, D. Stoyanov, G.-Z. Yang, "Metric Depth Recovery from Monocular Images Using Shape-from-Shading and Specularities", in IEEE International Conference on Image Processing (ICIP) 2012, to appear, Orlando, FL, 2012.
//
// For questions or comments please contact the author via the website at: http://www.commsp.ee.ic.ac.uk/~marcovs/
//////////////////////////////////////////////////////////////


#include "ShapeFromShading.h"

CShapeFromShading::CShapeFromShading(float albedo)
{
	cvNamedWindow("Brightness Image");
	cvNamedWindow("Groundtruth Depth");
	cvNamedWindow("Reprojected Image");
	cvNamedWindow("Reconstructed Depth");
	
	_sigmaD = albedo;
	
}

CShapeFromShading::~CShapeFromShading(void)
{
	delete [] _depth;
	delete [] _intImage;
	delete [] _v;
	delete [] _recDepth;
}

void CShapeFromShading::loadCamera(int height, int width, float xc, float yc, float f) {
	_width = width;
	_height = height;
	_depth = new float[_width*_height];
	_recDepth = new float[_width*_height];
	_intImage = new float[_width*_height];
	_v = new float[_width*_height];
	_epsilon = 0.00001;
	_f = f;
	_a = 0;
	_b = 0;
	_c = 0;
	_xc = xc;
	_yc = yc;
}

void CShapeFromShading::loadDepthFile(const char *fileName) {
	FILE *fileIn;
	fileIn = fopen(fileName,"r");
	if (!fileIn)
		return;
	float entry =0;
	_maxDepth = MININT;
	_minDepth = MAXINT;
	for (int i = 0; i < _height; i++) {
		for (int j = 0; j < _width; j++) {
			fscanf(fileIn,"%f\n",&entry);
			//entry /= _f; //u(x) = r(x)/f
			_depth[i*_width + j] = entry;
			_maxDepth = (entry > _maxDepth) ? entry : _maxDepth;
			_minDepth = (entry < _minDepth) ? entry : _minDepth;
		}
	}
	fclose(fileIn);
	generateIntensityImage();
	_hasGroundTruth = true;
}

void CShapeFromShading::loadImageFromFile(const char *fileName) {
	//image normalised between 0 and 1
	_inputRGB = cvLoadImage(fileName);
	if (_inputRGB->nChannels == 1) {
		for (int i = 0; i < _height; i++) {
			for (int j = 0; j < _width; j++) {
				uchar *imPtr = &CV_IMAGE_ELEM(_inputRGB,uchar,i,j);
				_intImage[i*_width+j] = imPtr[0];
			}
		}
	}
	else {
		for (int i = 0; i < _height; i++) {
			for (int j = 0; j < _width; j++) {
				uchar *imPtr = &CV_IMAGE_ELEM(_inputRGB,uchar,i,j*3);
				//it's actually BGR
				_intImage[i*_width+j] = (0.114*(float)imPtr[0] + 0.587*(float)imPtr[1] + 0.299*(float)imPtr[2]);
				//black shadow removal
				_intImage[i*_width+j] = (_intImage[i*_width+j] < 5) ? 5.0 : _intImage[i*_width+j];
				
			}
		}
	}
	
	_maxDepth = -1;
	_minDepth = -1;
	_hasGroundTruth = false;
}


void CShapeFromShading::saveDepthMap(const char *fileName) {
	ofstream fileOut;
	fileOut.open(fileName);
	for (int i = 0; i < _height; i++) {
		for (int j = 0; j < _width; j++) {
			fileOut << _recDepth[i*_width +j] << endl;
		}
	}
	fileOut.close();
}

void CShapeFromShading::changeLightParameters(float newA, float newB, float newC) {
	_a = newA;
	_b = newB;
	_c = newC;
}

void CShapeFromShading::generateIntensityImage() {
	float ux;
	float x, y, dux_x, dux_y, du_x, du_y;
	float dux_norm, x_norm;

	for (int i = 0; i < _height; i++) {
		for (int j = 0; j < _width; j++) {
			int idx = i*_width + j;
			ux = log(_depth[idx]);
			x = j - _xc;
			y = i - _yc;
			du_x = horizontalDevMin(i,j,_depth);
			du_y = verticalDevMin(i,j,_depth);
			dux_x = logHorizontalDevMin(i,j,_depth);
			dux_y = logVerticalDevMin(i,j,_depth);
			dux_norm = sqrt(dux_x*dux_x + dux_y*dux_y);
			x_norm = sqrt(x*x + y*y);

			float xfTerm = x_norm*x_norm + _f*_f;


			float intensity = (_f+_c)/(sqrt(pow(_f+_c,2)*(dux_x*dux_x + dux_y*dux_y) + pow(dux_x*(x+_a)+dux_y*(y+_b) + 1,2))*sqrt(pow(_f+_c,2) + pow(x+_a,2) + pow(y+_b,2)));
			float attenuation = exp(2*ux)*(pow((x+_a),2) + pow((y+_b),2) + pow((_f+_c),2));
			_intImage[idx] = _sigmaD*255*intensity/attenuation;

		}
	}
	
}

void CShapeFromShading::showImage() {
	IplImage *brightPix = cvCreateImage(cvSize(_width,_height),IPL_DEPTH_8U,1);
	cvSetZero(brightPix);
	for (int i = 0; i < _height; i++) {
		for (int j = 0; j < _width; j++) {
			uchar *imPtr = &CV_IMAGE_ELEM(brightPix,uchar,i,j);
			int intensity = cvRound(_intImage[i*_width + j]); 	
			imPtr[0] = intensity;
		}
	}
	
	cvShowImage("Brightness Image",brightPix);
	cvSaveImage("intensity.bmp",brightPix);
	cvWaitKey(1);
	cvReleaseImage(&brightPix);
}

void CShapeFromShading::showGroundTruth() {
	if (_hasGroundTruth) {
	IplImage *groundTruth = cvCreateImage(cvSize(_width,_height),IPL_DEPTH_8U,3);
	cvSetZero(groundTruth);
	for (int i = 0; i < _height; i++) {
		for (int j = 0; j < _width; j++) {
			int idx = i*_width + j;
			uchar *imPtr = &CV_IMAGE_ELEM(groundTruth, uchar, i, j*3);
			imPtr[0] = _R[cvRound(1023.0*(_depth[idx] - _minDepth)/(_maxDepth - _minDepth))];
			imPtr[1] = _G[cvRound(1023.0*(_depth[idx] - _minDepth)/(_maxDepth - _minDepth))];
			imPtr[2] = _B[cvRound(1023.0*(_depth[idx] - _minDepth)/(_maxDepth - _minDepth))];
		}
	}
	cvShowImage("Groundtruth Depth", groundTruth);
	cvSaveImage("gTruth.bmp",groundTruth);
	cvWaitKey(1);
	cvReleaseImage(&groundTruth);
	}
}

void CShapeFromShading::showReconstructedDepth() {
	IplImage *reconstructed = cvCreateImage(cvSize(_width,_height),IPL_DEPTH_8U,3);
	cvSetZero(reconstructed);
		for (int i = 0; i < _height; i++) {
			for (int j = 0; j < _width; j++) {
				int idx = i*_width + j;
				uchar *imPtr = &CV_IMAGE_ELEM(reconstructed, uchar, i, j*3);
				float recDepth = max(_recMin,min(_recMax,_recDepth[idx]));
				imPtr[0] = _R[cvRound(1023.0*(recDepth - _recMin)/(_recMax - _recMin))];
				imPtr[1] = _G[cvRound(1023.0*(recDepth - _recMin)/(_recMax - _recMin))];
				imPtr[2] = _B[cvRound(1023.0*(recDepth - _recMin)/(_recMax - _recMin))];
			}
		}
	cvShowImage("Reconstructed Depth", reconstructed);
	cvWaitKey(1);
	cvReleaseImage(&reconstructed);
}

void CShapeFromShading::showReprojectedImage() {
	float ux;
	float x, y, dux_x, dux_y;
	float dux_norm, x_norm;

	IplImage *brightPix = cvCreateImage(cvSize(_width,_height),IPL_DEPTH_8U,1);
	cvSetZero(brightPix);
	for (int i = 0; i < _height; i++) {
		for (int j = 0; j < _width; j++) {
			int idx = i*_width + j;
			ux = _recDepth[idx];
			x = j - _xc;
			y = i - _yc; 
			dux_x = horizontalDevMin(i,j,_recDepth);
			dux_y = verticalDevMin(i,j,_recDepth);
			dux_norm = sqrt(dux_x*dux_x + dux_y*dux_y);
			x_norm = sqrt(x*x + y*y);

			float xfTerm = x_norm*x_norm + _f*_f;
			float intensity = 0;

			ux = log(_depth[idx]);
			dux_x = logHorizontalDevMin(i,j,_depth);
			dux_y = logVerticalDevMin(i,j,_depth);

			intensity = (_f+_c)/(sqrt(pow(_f+_c,2)*(dux_x*dux_x + dux_y*dux_y) + pow(dux_x*(x+_a)+dux_y*(y+_b) + 1,2))*sqrt(pow(_f+_c,2) + pow(x+_a,2) + pow(y+_b,2)));
			float attenuation = exp(2*ux)*(pow((x+_a),2) + pow((y+_b),2) + pow((_f+_c),2));
			intensity = _sigmaD*255*intensity/attenuation;

			uchar *imPtr = &CV_IMAGE_ELEM(brightPix,uchar,i,j);
			imPtr[0] = cvRound(intensity); 
		}
	}
	
	cvShowImage("Reprojected Image",brightPix); 
	cvWaitKey(1);
	cvReleaseImage(&brightPix);
}

float CShapeFromShading::horizontalDevMin(int i, int j, float *arr) {
	int idx = i*_width +j;
	float prev = (j == 0) ? arr[idx] : arr[idx-1];
	float next = (j == (_width -1)) ? arr[idx] : arr[idx +1];
	float prevH = (j > 0) ? -arr[i*_width + j -1] + arr[i*_width + j] : 0;
 	float nextH = (j < (_width -1)) ? arr[i*_width + j +1] - arr[i*_width + j] : 0;
	float minimum = min(next-arr[idx],-prev+arr[idx]);
	minimum = (abs(nextH) < abs(prevH)) ? nextH : prevH;
	return minimum;
}

float CShapeFromShading::logHorizontalDevMin(int i, int j, float *arr) {
	int idx = i*_width +j;
	float prev = (j == 0) ? log(arr[idx]) : log(arr[idx-1]);
	float next = (j == (_width -1)) ? log(arr[idx]) : log(arr[idx +1]);
	float prevH = (j > 0) ? -log(arr[idx -1]) + log(arr[idx]) : 0;
 	float nextH = (j < (_width -1)) ? log(arr[idx +1]) - log(arr[idx]) : 0;
	float minimum = min(next-log(arr[idx]),-prev+log(arr[idx]));
	minimum = (abs(nextH) < abs(prevH)) ? nextH : prevH;
	return minimum;
}

float CShapeFromShading::verticalDevMin(int i, int j, float *arr) {
	int idx = i*_width +j;
	float prev = (i == 0) ? arr[idx] : arr[idx-_width];
	float next = (i == (_height -1)) ? arr[idx] : arr[idx +_width];
	float prevV = (i > 0) ? -arr[( i-1)*_width + j] + arr[i*_width + j] : 0;
 	float nextV = (i < (_height -1)) ? arr[(i+1)*_width + j] - arr[i*_width + j] : 0;
	float minimum = min(next-arr[idx],-prev+arr[idx]);
	minimum = (abs(nextV) < abs(prevV)) ? nextV : prevV;
	return minimum;
}

float CShapeFromShading::logVerticalDevMin(int i, int j, float *arr) {
	int idx = i*_width +j;
	float prev = (i == 0) ? log(arr[idx]) : log(arr[idx-_width]);
	float next = (i == (_height -1)) ? log(arr[idx]) : log(arr[idx +_width]);
	float prevV = (i > 0) ? -log(arr[( i-1)*_width + j]) + log(arr[i*_width + j]) : 0;
 	float nextV = (i < (_height -1)) ? log(arr[(i+1)*_width + j]) - log(arr[i*_width + j]) : 0;
	float minimum = min(next-log(arr[idx]),-prev+log(arr[idx]));
	minimum = (abs(nextV) < abs(prevV)) ? nextV : prevV;
	return minimum;
}

void CShapeFromShading::initialiseLF(float maxDepth) {
	//populate initial depth map
	for (int i = 0; i < _height; i++) {
		for (int j = 0; j < _width; j++) {
			int idx = i*_width+j;
			if ((i == 0) || (i == (_height-1)) || (j == 0) || (j == _width -1)) {
				 _v[idx] = (_hasGroundTruth) ? log(_depth[idx]) :  log(sqrt(255*_sigmaD/(_intImage[idx]*_f*_f)));
			}
			else
				_v[idx] = log(maxDepth);
		}
	}
}

void CShapeFromShading::iterateLF() {
	//sweep in four directions
	for (int sy = -1; sy <= 1; sy += 2) {
		for (int sx = -1; sx <= 1; sx += 2) {
			//update depth
			for (int i = ((sy < 0) ? _height -2 : 1); ((sy < 0) ? i >= 1 : i <= _height -2); i += sy) {
				for (int j = ((sx < 0) ? _width -2 : 1); ((sx < 0) ? j >= 1 : j <= _width -2); j += sx) {
					int idx = i*_width + j;
					float x = j - _xc;
					float y = i - _yc;
	
					
					//derivative terms in the hamiltonian
					float dv_x = (_v[idx+1] - _v[idx-1])/2.0;
					float dv_y = (_v[idx+_width] - _v[idx-_width])/2.0;
					
					float R,H, newv;
					float sax, say;
					float ux_0, ux_1, uy_0, uy_1;
					float ux_minus, ux_plus, uy_minus, uy_plus;
					ux_minus = _v[idx] - _v[idx-1]; ux_plus = _v[idx+1] - _v[idx];
					uy_minus = _v[idx] - _v[idx-_width]; uy_plus = _v[idx+_width] - _v[idx];
					float ip = _intImage[idx]*sqrt((_f+_c)*(_f+_c) + (x+_a)*(x+_a) + (y+_b)*(y+_b))*((_f+_c)*(_f+_c) + (x+_a)*(x+_a) + (y+_b)*(y+_b))/((_f + _c)*_sigmaD*100); //min between 100-150, fast at 230 with mozart
					ux_0 = ip*(ux_minus*((_f+_c)*(_f+_c) + (x+_a)*(x+_a)) + uy_minus*(x+_a)*(y+_b) + (x+_a))/sqrt((_f+_c)*(_f+_c)*(ux_minus*ux_minus + uy_minus*uy_minus) + (ux_minus*(x+_a) + uy_minus*(y+_b) + 1)*(ux_minus*(x+_a) + uy_minus*(y+_b) + 1));
					ux_1 = ip*(ux_plus*((_f+_c)*(_f+_c) + (x+_a)*(x+_a)) + uy_plus*(x+_a)*(y+_b) + (x+_a))/sqrt((_f+_c)*(_f+_c)*(ux_plus*ux_plus + uy_plus*uy_plus) + (ux_plus*(x+_a) + uy_plus*(y+_b) + 1)*(ux_plus*(x+_a) + uy_plus*(y+_b) + 1));
					uy_0 = ip*(uy_minus*((_f+_c)*(_f+_c) + (y+_b)*(y+_b)) + ux_minus*(x+_a)*(y+_b) + (y+_b))/sqrt((_f+_c)*(_f+_c)*(ux_minus*ux_minus + uy_minus*uy_minus) + (ux_minus*(x+_a) + uy_minus*(y+_b) + 1)*(ux_minus*(x+_a) + uy_minus*(y+_b) + 1));
					uy_1 = ip*(uy_plus*((_f+_c)*(_f+_c) + (y+_b)*(y+_b)) + ux_plus*(x+_a)*(y+_b) + (y+_b))/sqrt((_f+_c)*(_f+_c)*(ux_plus*ux_plus + uy_plus*uy_plus) + (ux_plus*(x+_a) + uy_plus*(y+_b) + 1)*(ux_plus*(x+_a) + uy_plus*(y+_b) + 1));
					
					ux_0 = fabs(ux_0);
					ux_1 = fabs(ux_1);
					uy_0 = fabs(uy_0);
					uy_1 = fabs(uy_1);

					sax = (ux_0 > ux_1) ? ux_0 : ux_1;
					say = (uy_0 > uy_1) ? uy_0 : uy_1;

	
					R = 0;
					float s = 512;
					H = (1/(255*_sigmaD))*_intImage[idx]*sqrt((_f+_c)*(_f+_c)*(dv_x*dv_x + dv_y*dv_y) + (dv_x*(x+_a) + dv_y*(y+_b) +1)*(dv_x*(x+_a) + dv_y*(y+_b) +1))*sqrt((_f+_c)*(_f+_c) + (x+_a)*(x+_a) + (y+_b)*(y+_b))*((_f+_c)*(_f+_c) + (x+_a)*(x+_a) + (y+_b)*(y+_b))/(_f+_c);
	
					float a = -(1/(sax+say));
					float b = (1/(sax+say))*(R - H + sax*(_v[idx+1] + _v[idx-1])/2.0 + say*(_v[idx+_width] + _v[idx-_width])/2.0);

					//Newton's method to fit the exponential term
					
					newv = _v[idx];
					for (int p = 0; p < 10; p++) {
						float expTerm = exp(-2*newv);
						newv = newv - (a*expTerm + newv - b)/(-2*a*expTerm+1);
					}


					
					_v[idx] = (newv < _v[idx]) ? newv : _v[idx];
				}
			}

			//check boundaries
			//columns
			for (int i = 0; i < _height; i++) {
				int idx0 = i*_width;
				int idx1 = i*_width + _width-1;
				_v[idx0] = min(max(2*_v[idx0 +1] - _v[idx0+2],_v[idx0+2]),_v[idx0]);
				_v[idx1] = min(max(2*_v[idx1 -1] - _v[idx1-2],_v[idx1-2]),_v[idx1]); 
			}
			//rows
			for (int j = 0; j < _width; j++) {
				int idx0 = j;
				int idx1 = (_height-1)*_width + j;
				_v[idx0] = min(max(2*_v[idx0 + _width] - _v[idx0 + 2*_width],_v[idx0 + 2*_width]),_v[idx0]);
				_v[idx1] = min(max(2*_v[idx1 - _width] - _v[idx1 - 2*_width],_v[idx1 - 2*_width]),_v[idx1]);
			}
		}
	}
}

void CShapeFromShading::reconstructSurface(float maxDepth, int maxIter) {
	initialiseLF(maxDepth);
	int iter = 0;
	memset(_recDepth,0,_width*_height*sizeof(float));
	while(true) {
		_recMax = MININT;
		_recMin = MAXINT;
		iterateLF();
	
		for (int i = 0; i < _height; i++) { 
			for (int j = 0; j < _width; j++) {

				_recDepth[i*_width +j] = exp(_v[i*_width+j]);

				_recMax = (_recMax < _recDepth[i*_width+j]) ? _recDepth[i*_width+j] : _recMax;
				_recMin = (_recMin > _recDepth[i*_width+j]) ? _recDepth[i*_width+j] : _recMin; 
			}
		}
 		showReconstructedDepth();
		//showReprojectedImage();

		cout << "Iteration: " << iter << endl;
		int key = cvWaitKey(1);
		if ((key != -1) || ((maxIter != 0) && (iter == maxIter)))
			break;
		iter++;
	}
	float err = 0;
	float percErr = 0;
	computeError(err,percErr);
	//When computing the error on synthetic images, make sure to exclude the background (not implemented here)
	cout << "Error: " << percErr << "%" << endl;
}

void CShapeFromShading::computeError(float &err, float &percErr) {
	if (_hasGroundTruth) {
	for (int i = 0; i < _height; i++) {
		for (int j = 0; j < _width; j++) {
			float pixelErr = abs(_depth[i*_width+j] - _recDepth[i*_width+j]);
			err += pixelErr;
			percErr += pixelErr/_depth[i*_width];
		}
	}
	err = err/(_width*_height);
	percErr = abs(percErr*100/(_width*_height));
	}
}
