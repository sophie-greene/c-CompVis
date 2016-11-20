/**
* @brief Stereo Block Match body in OF
* @file block_match.hpp
* @date 26/04/2012
*
*/


#include "block_match.hpp"

using namespace std;
using namespace boost;
using namespace cv;

void BlockMatch::setup(GlobalConfig &config) {
	
	mObj.reset(new SharedObj(config));
	mObj->mShader.load("block_match");

	// Disparity map so using h/w not w/h
	
	mObj->mDimY = 1;
	while (mObj->mDimY < mObj->mConfig.camSize.width){
		mObj->mDimY = mObj->mDimY << 1;
	}
	
	mObj->mDimX = 1;
	while ( mObj->mDimX < mObj->mConfig.camSize.height){
		mObj->mDimX = mObj->mDimX << 1;
	}

	mObj->mFbo0.allocate(mObj->mDimX, mObj->mDimY, GL_RGB);
	
	// Initial setup of openGL textures
	
	Mat camA (Size(mObj->mDimX, mObj->mDimY),CV_8UC3);
	Mat camB (Size(mObj->mDimX, mObj->mDimY),CV_8UC3);
	
	mObj->mT0 = matToTexture(camA,GL_NEAREST_MIPMAP_LINEAR,GL_LINEAR,GL_CLAMP);
	mObj->mT1 = matToTexture(camB,GL_NEAREST_MIPMAP_LINEAR,GL_LINEAR,GL_CLAMP);


}

// Function turn a cv::Mat into a texture, and return the texture ID as a GLuint for use
GLuint BlockMatch::matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter)
{
	// Generate a number for our textureID's unique handle
	GLuint textureID;
	glGenTextures(1, &textureID);
 
	// Bind to our texture handle
	glBindTexture(GL_TEXTURE_2D, textureID);
 
	// Catch silly-mistake texture interpolation method for magnification
	if (magFilter == GL_LINEAR_MIPMAP_LINEAR  ||
	    magFilter == GL_LINEAR_MIPMAP_NEAREST ||
	    magFilter == GL_NEAREST_MIPMAP_LINEAR ||
	    magFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
		magFilter = GL_LINEAR;
	}
 
	// Set texture interpolation methods for minification and magnification
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
 
	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);
 
	// Set incoming texture format to:
	// GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
	// GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
	// Work out other mappings as required ( there's a list in comments in main() )
	GLenum inputColourFormat = GL_RGB;
	if (mat.channels() == 1)
	{
		inputColourFormat = GL_LUMINANCE;
	}
 
	// Create the texture
	glTexImage2D(GL_TEXTURE_2D,     // Type of texture
	             0,                 // Pyramid level (for mip-mapping) - 0 is the top level
	             GL_RGB,            // Internal colour format to convert to
	             mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
	             mat.rows,          // Image height i.e. 480 for Kinect in standard mode
	             0,                 // Border width in pixels (can either be 1 or 0)
	             inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
	             GL_UNSIGNED_BYTE,  // Image data type
	             mat.ptr());        // The actual image data itself
 
 
	// If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
	if (minFilter == GL_LINEAR_MIPMAP_LINEAR  ||
	    minFilter == GL_LINEAR_MIPMAP_NEAREST ||
	    minFilter == GL_NEAREST_MIPMAP_LINEAR ||
	    minFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		glGenerateMipmap(GL_TEXTURE_2D);
	}
 
	return textureID;
}

// Could potentially use mipmaps and use a hierarchical approach? Might be tricky with textureRect

/*
 * Match the two images using a shader. Images should be RGB
 * Shift the second image around and record the difference. Always keep the lower difference
 */

ofTexture& BlockMatch::match(cv::Mat m0, cv::Mat m1) {
		
	float w = m0.cols;
	float h = m0.rows;
	
	Mat camA (Size(mObj->mDimX, mObj->mDimY),CV_8UC3);
	Mat camB (Size(mObj->mDimX, mObj->mDimY),CV_8UC3);
	
	Mat camROIA = camA(Rect(0, 0, m0.cols, m0.rows));
	Mat camROIB = camB(Rect(0, 0, m0.cols, m0.rows));
	
	m0.copyTo(camROIA);
	m1.copyTo(camROIB);
	
	mObj->mFbo0.begin();
	ofClear(0, 0, 0); 
			
	mObj->mShader.begin();
	
	glBindTexture(GL_TEXTURE_2D, mObj->mT0);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, camA.cols, camA.rows, GL_RGB, GL_UNSIGNED_BYTE, camA.ptr());
	glGenerateMipmap(GL_TEXTURE_2D);
	
	glBindTexture(GL_TEXTURE_2D, mObj->mT1);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, camA.cols, camA.rows, GL_RGB, GL_UNSIGNED_BYTE, camB.ptr());
	glGenerateMipmap(GL_TEXTURE_2D);
	
	mObj->mShader.setUniformTexture("camTexA", GL_TEXTURE_2D, mObj->mT0, 0);
	mObj->mShader.setUniformTexture("camTexB", GL_TEXTURE_2D, mObj->mT1, 1);
		
	mObj->mShader.setUniform1i("rangeX", mObj->mConfig.blockSize);
	mObj->mShader.setUniform1i("rangeY", mObj->mConfig.blockSize);
	mObj->mShader.setUniform1i("stepSize", mObj->mConfig.stepSize);
	
	float t = mObj->mConfig.blockSize * mObj->mConfig.blockSize;
	mObj->mShader.setUniform1f("offsetL", 1.0 / sqrt(t) );
	
	mObj->mShader.setUniform1i("width", w);
	mObj->mShader.setUniform1i("height", h);

	mObj->mShader.setUniform1f("topLod", (float)mObj->mConfig.lod);
			
	float tx =(float)w/ (float)mObj->mDimX;
	float ty =(float)h/ (float)mObj->mDimY;	

			
	glBegin(GL_QUADS);
	glTexCoord2f(0, 0); 	glVertex3f(-1, -1, 0);
	glTexCoord2f(tx, 0); 	glVertex3f(1, -1, 0);
	glTexCoord2f(tx,ty); 	glVertex3f(1, 1, 0);
	glTexCoord2f(0, ty);	glVertex3f(-1,1, 0);
	glEnd();
	
	mObj->mShader.end(); 
	mObj->mFbo0.end(); 
			

	return mObj->mFbo0.getTextureReference();
}
