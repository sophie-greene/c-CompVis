/////////////////////////////////////////////////////////////
// Name: SFSmain.cpp - main test file
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


int main(void) {
	
	//Workflow 1: Load ground truth depth from file -> generate intensity image -> estimate depth -> compute error
	CShapeFromShading sfs = CShapeFromShading(1500000000);
	sfs.loadCamera(256,256,128,128,300);
	sfs.loadDepthFile("./TestData/mozart.txt");
	sfs.reconstructSurface(400,200);

	//Workflow 2: Load RGB image -> estimate depth
	//CShapeFromShading sfs = CShapeFromShading(1); //constructor with albedo value
	//sfs.loadCamera(200,200,100,100,300);
	//sfs.changeLightParameters(0,0,0);
	//sfs.loadImageFromFile("./TestData/gastrolab.bmp");
	//sfs.reconstructSurface(400,200);

	//Show results and save output depth map
	sfs.showGroundTruth();
	sfs.showImage();
	sfs.showReconstructedDepth();
	sfs.saveDepthMap("./TestData/reconstructedDepth.txt");
	cvWaitKey();

	return 0;
}
