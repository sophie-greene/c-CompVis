/**
* @brief Main Program based header
* @file leeds.hpp
* @date 26/04/2012
*
*/

// http://opencv.itseez.com/modules/gpu/doc/camera_calibration_and_3d_reconstruction.html
// http://stackoverflow.com/questions/5987285/what-is-an-acceptable-return-value-from-cvcalibratecamera


#ifndef _LEEDS_HPP_
#define _LEEDS_HPP_

#include "tinyxml.h"

#include <stdint.h> 

#include <opencv/highgui.h>
#include <gpu/gpu.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include "camera_manager.hpp"
#include "utils.hpp"
#include "config.hpp"
#include "drawer.hpp"
#include "states.hpp"
#include "app.hpp"

#undef Success //Needed for PCL stuff I believe?

#include "mesh.hpp"



/*
 * Small class to hold a reference between name and function
 */
 
class CameraControlRef {
public:
	CameraControlRef(std::string n, boost::function<void (int x)> f ){ name = n; callBack = f; };
	
	boost::function<void (unsigned int x)> callBack;
	std::string name;
};




/*
 * Main class that parses input and loads the other classes for
 * calibration and similar. Exports to the UI our functionality
 */
 

class Leeds : public AppProvider {
	
public:
	Leeds();
	~Leeds();

	void draw(double_t dt);
	
	void init();
	void resize(int w, int h);
	void keyCallback(int key, int action);
	void mousePositionCallback(int x, int y);
	void mouseButtonCallback(int button, int action);
	void mouseWheelCallback(int xpos, int ypos);
	

	void parseXML();
			
	// Toggleable states for QT and other UI hooks
	void toggleShowCameras();
	void toggleScanning();
	void toggleDetected();
	void toggleDrawMesh();
	void toggleTexturing();
	void clearMesh();
	void generateMesh();
	void calibrateCameras();
	void calibrateWorld();
	std::string getStatus() { if (pInfo) return pInfo->getStatus(); return "Leeds - Nonestate"; } 
	void save();
	void quit();
	void load(std::string filename="./data/test.pcd");


	// Small Callbacks for UVC Cameras - exposed for UI
	void brightness(int value){setAllCameras(BRIGHTNESS,(unsigned int) value); };
	void contrast(int value){ setAllCameras(CONTRAST,(unsigned int) value); };
	void saturation(int value){ setAllCameras(SATURATION,(unsigned int) value); };
	void gain(int value){ setAllCameras(GAIN,(unsigned int) value); };
	void sharpness(int value){ setAllCameras(SHARPNESS,(unsigned int) value); };
	void autoexposure(int value){ setAllCameras(AUTO_EXPOSURE,(unsigned int) value); };
	void exposure(int value){ setAllCameras(EXPOSURE,(unsigned int) value); };
	void focus(int value){ setAllCameras(FOCUS,(unsigned int) value); };
	void autofocus(int value){ setAllCameras(AUTO_FOCUS,(unsigned int) value); };
	
	GlobalConfig mConfig;
	
protected:

	void _update(); 


	SharedInfo pInfo;	// Holds the state which we pass to the running states

	// State Control
	// Annoyingly we can't have copy/value semantics here - STL copies only the base class portion
	
	LeedsQueue qState;
	LeedsList lState;	

	// Clock timing functions
	void startClock(int idx) { gettimeofday (&start[idx],NULL);} ;
	double sampleClock(int idx) { 
			gettimeofday(&end[idx],NULL); 
			double ds = (start[idx].tv_sec * 1000.0) + (start[idx].tv_usec * 0.001);
			double de = (end[idx].tv_sec * 1000.0) + (end[idx].tv_usec * 0.001);
			double d = de - ds; 
			gettimeofday(&start[idx],NULL);
			return d * 0.001; 
		};


	// Controls all the camera functions and basic calibration features
	CameraManager mManager;
	
	// GUI
	std::vector<CameraControlRef> vCamControls;
	bool mDG;
	
	// Drawer - does all the drawing
	Drawer mD;
	
	// Mesh class that does a lot of hard work with the points
	LeedsMesh mM;
		   
    // Main Loop Functions
    bool mGo;
    boost::thread *pUpdateThread;
    
	TwBar *mMainBar;
	
	// call to set all the connected cameras
	void setAllCameras(CameraControl c, unsigned int v);

	timeval start[5],end[5];

	
	MouseState mMouse;


};

#endif
