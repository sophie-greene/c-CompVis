/**
* @brief Misc Functions that appear in various places
* @file utils.cpp
* @date 03/05/2012
*
*/

#include "utils.hpp"

using namespace std;
using namespace cv;

/*
 * Find Chessboards in the Matrix
 */

bool findChessboard(Mat &cam0, vector<Point2f> &corners, Mat &board, cv::Size &size ) {

	if ( findChessboardCorners(cam0, size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK) ) {
	
		// Update the view
		
		cam0.copyTo(board);
		drawChessboardCorners(board, size, corners, true);
		
		Mat grey = cam0;

		cvtColor( cam0, grey, CV_RGB2GRAY);		
		cornerSubPix(grey, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));

		return true;
	}

	cam0.copyTo(board);	
	return false;
}


/*
 * Load Intrinsic Parameters from disk
 */


bool loadCameraParameters(string filename, CameraParameters &ip) {
	try {
		FileStorage fs(filename.c_str(), CV_STORAGE_READ);
		if(!fs.isOpened()) {
			cout << "Leeds - Failed to open intrinsic file "  << filename << endl;
			return false;
		}

		fs["M"] >> ip.M;
		fs["D"] >> ip.D;

		try {
			fs["R"] >> ip.R;
			fs["T"] >> ip.T;
		}
		catch (...) {
			cout << "Leeds - failed to load world transforms in intrinsics." << endl;
		}
		
		cout << "Leeds - Loaded camera Parameters " << filename << endl;
		ip.mCalibrated = true;
		fs.release();
		return true;
		
	} catch(...) {
		cout << "Leeds - failed to load intrinsic variables. You will need to calibrate." << endl;
	}

	return false;
}


/*
 * Save the Intrinsic Parameters to a set of files
 */

bool saveCameraParameters(string filename, CameraParameters &in) {
	

    FileStorage fs(filename, CV_STORAGE_WRITE);
    if( fs.isOpened() ) {
        fs << "M" << in.M << "D" << in.D << "R" << in.R << "T" << in.T;
        fs.release();
        return true;
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";
    
    return false;
}


std::string getTextureParameters(GLuint id)
{
    if(glIsTexture(id) == GL_FALSE)
        return "Not texture object";

    int width, height, format;
    std::string formatName;
    glBindTexture(GL_TEXTURE_2D, id);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);            // get texture width
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);          // get texture height
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &format); // get texture internal format
    glBindTexture(GL_TEXTURE_2D, 0);

    formatName = convertInternalFormatToString(format);

    std::stringstream ss;
    ss << width << "x" << height << ", " << formatName;
    return ss.str();
}


std::string getRenderbufferParameters(GLuint id)
{
    if(glIsRenderbuffer(id) == GL_FALSE)
        return "Not Renderbuffer object";

    int width, height, format;
    std::string formatName;
    glBindRenderbuffer(GL_RENDERBUFFER, id);
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &width);    // get renderbuffer width
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &height);  // get renderbuffer height
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_INTERNAL_FORMAT, &format); // get renderbuffer internal format
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    formatName = convertInternalFormatToString(format);

    std::stringstream ss;
    ss << width << "x" << height << ", " << formatName;
    return ss.str();
}


std::string convertInternalFormatToString(GLenum format)
{
    std::string formatName;

    switch(format)
    {
    case GL_STENCIL_INDEX:
        formatName = "GL_STENCIL_INDEX";
        break;
    case GL_DEPTH_COMPONENT:
        formatName = "GL_DEPTH_COMPONENT";
        break;
    case GL_DEPTH_STENCIL:
        formatName = "GL_DEPTH_STENCIL";
        break;
    case GL_ALPHA:
        formatName = "GL_ALPHA";
        break;
    case GL_RGB:
        formatName = "GL_RGB";
        break;
    case GL_RGBA:
        formatName = "GL_RGBA";
        break;
    case GL_LUMINANCE:
        formatName = "GL_LUMINANCE";
        break;
    case GL_LUMINANCE_ALPHA:
        formatName = "GL_LUMINANCE_ALPHA";
        break;
    case GL_ALPHA4:
        formatName = "GL_ALPHA4";
        break;
    case GL_ALPHA8:
        formatName = "GL_ALPHA8";
        break;
    case GL_ALPHA12:
        formatName = "GL_ALPHA12";
        break;
    case GL_ALPHA16:
        formatName = "GL_ALPHA16";
        break;
    case GL_LUMINANCE4:
        formatName = "GL_LUMINANCE4";
        break;
    case GL_LUMINANCE8:
        formatName = "GL_LUMINANCE8";
        break;
    case GL_LUMINANCE12:
        formatName = "GL_LUMINANCE12";
        break;
    case GL_LUMINANCE16:
        formatName = "GL_LUMINANCE16";
        break;
    case GL_LUMINANCE4_ALPHA4:
        formatName = "GL_LUMINANCE4_ALPHA4";
        break;
    case GL_LUMINANCE6_ALPHA2:
        formatName = "GL_LUMINANCE6_ALPHA2";
        break;
    case GL_LUMINANCE8_ALPHA8:
        formatName = "GL_LUMINANCE8_ALPHA8";
        break;
    case GL_LUMINANCE12_ALPHA4:
        formatName = "GL_LUMINANCE12_ALPHA4";
        break;
    case GL_LUMINANCE12_ALPHA12:
        formatName = "GL_LUMINANCE12_ALPHA12";
        break;
    case GL_LUMINANCE16_ALPHA16:
        formatName = "GL_LUMINANCE16_ALPHA16";
        break;
    case GL_INTENSITY:
        formatName = "GL_INTENSITY";
        break;
    case GL_INTENSITY4:
        formatName = "GL_INTENSITY4";
        break;
    case GL_INTENSITY8:
        formatName = "GL_INTENSITY8";
        break;
    case GL_INTENSITY12:
        formatName = "GL_INTENSITY12";
        break;
    case GL_INTENSITY16:
        formatName = "GL_INTENSITY16";
        break;
    case GL_R3_G3_B2:
        formatName = "GL_R3_G3_B2";
        break;
    case GL_RGB4:
        formatName = "GL_RGB4";
        break;
    case GL_RGB5:
        formatName = "GL_RGB4";
        break;
    case GL_RGB8:
        formatName = "GL_RGB8";
        break;
    case GL_RGB10:
        formatName = "GL_RGB10";
        break;
    case GL_RGB12:
        formatName = "GL_RGB12";
        break;
    case GL_RGB16:
        formatName = "GL_RGB16";
        break;
    case GL_RGBA2:
        formatName = "GL_RGBA2";
        break;
    case GL_RGBA4:
        formatName = "GL_RGBA4";
        break;
    case GL_RGB5_A1:
        formatName = "GL_RGB5_A1";
        break;
    case GL_RGBA8:
        formatName = "GL_RGBA8";
        break;
    case GL_RGB10_A2:
        formatName = "GL_RGB10_A2";
        break;
    case GL_RGBA12:
        formatName = "GL_RGBA12";
        break;
    case GL_RGBA16:
        formatName = "GL_RGBA16";
        break;
    default:
        formatName = "Unknown Format";
    }

    return formatName;
}





