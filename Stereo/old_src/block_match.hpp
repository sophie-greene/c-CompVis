/**
* @brief Stereo Blokc Match Header in OF
* @file block_match.hpp
* @date 26/04/2012
*
*/

#ifndef __BLOCK_MATCH_HPP__
#define __BLOCK_MATCH_HPP__

 
#include <ofMain.h>
#include <boost/shared_ptr.hpp>
#include <opencv/highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "config.hpp"
#include "utils.hpp"

class BlockMatch {
public:
	BlockMatch() {};
	
	void setup(GlobalConfig &config);
	ofTexture & match(cv::Mat m0, cv::Mat m1);

protected:

	GLuint matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter);


	struct SharedObj {
	
		SharedObj(GlobalConfig &config) : mConfig(config) {};
		GlobalConfig &mConfig;
		ofShader mShader,mShader1; 
		ofFbo mFbo0;
		uint32_t mDimX, mDimY;
		int mT0, mT1;

	};
	
	boost::shared_ptr<SharedObj> mObj;
};


#endif
