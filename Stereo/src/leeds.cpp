/**
* @brief Leeds main program body
* @file leeds.cpp
* @date 26/04/2012
*
*/

#include "leeds.hpp"

using namespace cv;
using namespace std;
using namespace boost;

namespace po = boost::program_options;
	

/*
 * Set a control for all the cameras
 */

void Leeds::setAllCameras(CameraControl c, unsigned int v){
	mManager.setControl(c,v);
}


/*
 * Update Function - Called as often as possible to update state - threaded - renamed to not clash with QT
 */

void Leeds::_update(){
	
	startClock(0);
		
	pInfo->sr = false;
	
	double pt = 0;
	
	while (mGo){		
		
		double dt = sampleClock(0);
		pInfo->dt = dt;
				
		if (qState.size() > 0){
			if (qState.back().mF){
				qState.pop_back();
			}
			else
				qState.back().update();
		}
				
		// update the list states
		for (state_iterator it = lState.begin(); it != lState.end(); ++it){
			it->update();
		}
		
	}
	
}


/*
 * Exit function. Perform tidying up
 */

void Leeds::quit(){
	mGo = false;
	
	vector<string> fn;
	
	TiXmlDocument doc( "./data/settings.xml" );
	bool loadOkay = doc.LoadFile();
	
	if (loadOkay){
		
		try{
			TiXmlElement *pRoot = doc.FirstChildElement( "leeds" );
			
			TiXmlElement *pP;
			if ( pRoot ) {
				// Deal with the Cameras
				
				TiXmlElement *pCameras= pRoot->FirstChildElement("cameras");
				
				// Grab all the cameras
				TiXmlElement *pCam = pCameras->FirstChildElement("cam");
				while (pCam) {
					pP = pCam->FirstChildElement("in"); 	
					fn.push_back(string(pP->GetText()));
					pCam = pCam->NextSiblingElement();
				}
			}
		}
		catch(...){
			cerr << "Leeds - Error in saving" << endl;
		}
	}
	cout << "Leeds - Saving Camera Settings" << endl;
	mManager.saveSettings(fn);
	mManager.shutdown();
					
}

/// Leeds main class

Leeds::Leeds()  {
}

Leeds::~Leeds() {
}

void Leeds::parseXML(){
	TiXmlDocument doc( "./data/settings.xml" );
	bool loadOkay = doc.LoadFile();
	if (loadOkay){
		
		try{
		
			TiXmlElement *pRoot = doc.FirstChildElement( "leeds" );
			if ( pRoot ) {
				
				// Deal with the Cameras
				
				TiXmlElement *pCameras= pRoot->FirstChildElement("cameras");
				
				TiXmlElement *pP;
				// Grab basic camera details
				pP = pCameras->FirstChildElement("width"); mConfig.camSize.width = fromStringS9<int>(string(pP->GetText()));
				pP = pCameras->FirstChildElement("height"); mConfig.camSize.height = fromStringS9<int>(string(pP->GetText()));
				pP = pCameras->FirstChildElement("fps"); mConfig.fps = fromStringS9<int>(string(pP->GetText()));
				
				// Load the camera manager
				mManager.setup(mConfig);
				
				// Grab all the cameras
				TiXmlElement *pCam = pCameras->FirstChildElement("cam");
				while (pCam) {
					pP = pCam->FirstChildElement("dev");	string dev = string(pP->GetText());
					pP = pCam->FirstChildElement("in"); 	string in = string(pP->GetText());
					
					shared_ptr<LeedsCam> m0 = mManager.addCamera(dev,in);

					pCam = pCam->NextSiblingElement();
				}
				
				// Deal with the Chessboard
				TiXmlElement *pChess = pRoot->FirstChildElement("chess");
				
				pP = pChess->FirstChildElement("width"); mConfig.boardSize.width = fromStringS9<int>(string(pP->GetText()));
				pP = pChess->FirstChildElement("height"); mConfig.boardSize.height = fromStringS9<int>(string(pP->GetText()));
				//pP = pChess->FirstChildElement("size");
				pP = pChess->FirstChildElement("maximages"); mConfig.maxImages = fromStringS9<int>(string(pP->GetText()));
				pP = pChess->FirstChildElement("interval"); mConfig.interval = fromStringS9<float>(string(pP->GetText()));
				//pP = pChess->FirstChildElement("startpair"); 
				//pP = pChess->FirstChildElement("endpair");
				pP = pChess->FirstChildElement("startcam"); mConfig.startCam = fromStringS9<int>(string(pP->GetText()));
				pP = pChess->FirstChildElement("endcam"); mConfig.endCam = fromStringS9<int>(string(pP->GetText()));
				
				// Deal with World
				
				TiXmlElement *pWorld = pRoot->FirstChildElement("world");
				TiXmlElement *pSize = pWorld->FirstChildElement("size");
				
				pP = pSize->FirstChildElement("xs"); mConfig.xs = fromStringS9<float>(string(pP->GetText()));
				pP = pSize->FirstChildElement("ys"); mConfig.ys = fromStringS9<float>(string(pP->GetText()));
				pP = pSize->FirstChildElement("zs"); mConfig.zs = fromStringS9<float>(string(pP->GetText()));
				pP = pSize->FirstChildElement("xe"); mConfig.xe = fromStringS9<float>(string(pP->GetText()));
				pP = pSize->FirstChildElement("ye"); mConfig.ye = fromStringS9<float>(string(pP->GetText()));
				pP = pSize->FirstChildElement("ze"); mConfig.ze = fromStringS9<float>(string(pP->GetText()));
				
				TiXmlElement *pMesh = pWorld->FirstChildElement("mesh");
				pSize = pMesh->FirstChildElement("size");
				
				pP = pSize->FirstChildElement("x"); mConfig.meshResolution.x = fromStringS9<float>(string(pP->GetText()));
				pP = pSize->FirstChildElement("y"); mConfig.meshResolution.y = fromStringS9<float>(string(pP->GetText()));
				pP = pSize->FirstChildElement("z"); mConfig.meshResolution.z = fromStringS9<float>(string(pP->GetText()));
				
				// Deal with UVC
				
				TiXmlElement *pUVC = pRoot->FirstChildElement("uvc");
				
				pP = pUVC->FirstChildElement("brightness"); brightness(fromStringS9<int>(string(pP->GetText())));
				pP = pUVC->FirstChildElement("contrast"); contrast(fromStringS9<int>(string(pP->GetText())));
				pP = pUVC->FirstChildElement("saturation"); saturation(fromStringS9<int>(string(pP->GetText())));
				pP = pUVC->FirstChildElement("focus"); focus(fromStringS9<int>(string(pP->GetText())));
				pP = pUVC->FirstChildElement("exposure"); exposure(fromStringS9<int>(string(pP->GetText())));
				pP = pUVC->FirstChildElement("gain"); gain(fromStringS9<int>(string(pP->GetText())));
				pP = pUVC->FirstChildElement("sharpness"); sharpness(fromStringS9<int>(string(pP->GetText())));
				
				// Deal with Poisson
				TiXmlElement *pPoisson = pRoot->FirstChildElement("poisson");
				pP = pPoisson->FirstChildElement("depth"); mConfig.poissonDepth = fromStringS9<int>(string(pP->GetText()));
				pP = pPoisson->FirstChildElement("samples"); mConfig.poissonSamples = fromStringS9<float>(string(pP->GetText()));
				pP = pPoisson->FirstChildElement("scale"); mConfig.poissonScale = fromStringS9<float>(string(pP->GetText()));
				
				
				// Deal with PCL
				TiXmlElement *pPCL = pRoot->FirstChildElement("pcl");
				pP = pPCL->FirstChildElement("filterk"); mConfig.pclFilterMeanK = fromStringS9<float>(string(pP->GetText()));
				pP = pPCL->FirstChildElement("filterthresh"); mConfig.pclFilterThresh = fromStringS9<float>(string(pP->GetText()));
				pP = pPCL->FirstChildElement("searchradius"); mConfig.pclSearchRadius = fromStringS9<float>(string(pP->GetText()));
				pP = pPCL->FirstChildElement("polynomial"); mConfig.pclPolynomialOrder = fromStringS9<float>(string(pP->GetText()));
				pP = pPCL->FirstChildElement("sampleradius"); mConfig.pclUpsamplingRadius = fromStringS9<float>(string(pP->GetText()));
				pP = pPCL->FirstChildElement("stepsize"); mConfig.pclUpsamplingStepSize = fromStringS9<float>(string(pP->GetText()));
				
				
				// Deal with OpenCV
				TiXmlElement *pOpenCV = pRoot->FirstChildElement("opencv");
				pP = pOpenCV->FirstChildElement("threshold"); mConfig.pointThreshold = fromStringS9<float>(string(pP->GetText()));
				
			
			}
		}
		catch(...){
			cerr << "Leeds - Failed to load XML - quitting" << endl;
			exit(0);
		}
		
	}
	else{
		cerr << "Leeds - Failed to load ./data/settings.xml" << endl;

		exit(0);
	}
}


void Leeds::init(){
	
	TwWindowSize(mW, mH);
	mMainBar = TwNewBar("Leeds");
	
	
	parseXML();
	
	mMouse.mX = mMouse.mY = -1;
	mMouse.mState = 0;
	mMouse.mWheel = 0;
	
	mConfig.scanInterval = 20.0;
	
	// Add Toolbars
	
	TwAddVarRW(mMainBar, "Scan Speed", TW_TYPE_DOUBLE, &mConfig.scanInterval,
	"label='Scan Speed' min=1.0 max=100.0 step=1.0 help='Scan speed of the projector point'");
	
	
	// Fire up the drawer				
	mD.setup(mConfig);	
		
	// Mesh starting
	mM.setup(mConfig);

	pInfo = shared_ptr<StateInfo>(new StateInfo(mManager,mD,mM,mMouse));

	// Configure States
	StackState<BaseState> l(qState,pInfo);
	l();
		
	// Launch update thread
	mGo = true;
	pUpdateThread =  new boost::thread(&Leeds::_update, this);

    startClock(1);
}



/*
 * Resize function
 */

void Leeds::resize(int w, int h){
	
	mW = w;
	mH = h;
	
	cout << "LW " << mW << " " << mH << endl; 
	
	glViewport(0,0,mW,mH);
	
	if (qState.size() > 0){
		mD.resize(w,h);
	}
	TwWindowSize(mW, mH);
}

/*
 * Callback for the mouse position
 */

void Leeds::mousePositionCallback(int x, int y){
	
	if (mMouse.mX != -1 && mMouse.mY != -1){
		mMouse.mDX = x - mMouse.mX;
		mMouse.mDY = y - mMouse.mY;
		if (mMouse.mState & MOUSE_LEFT){
			mD.rotateCamera(mMouse.mDX,mMouse.mDY,0.01);
		}
		if (mMouse.mState & MOUSE_MIDDLE)
			mD.moveCamera(mMouse.mDX,mMouse.mDY,0.01);
	}
	mMouse.mX = x;
	mMouse.mY = y;
}

/*
 * Deal with mouse button state
 */

void Leeds::mouseButtonCallback(int button, int action){
	
	switch (button) {
		case 0:{
			if (action){
				mMouse.mState |= MOUSE_LEFT;
				if(mD.picked(mMouse.mX,mMouse.mY,16,glm::vec3(1.0,0.0,0.0))){
					mMouse.mState |= MOUSE_SELECTED;
				} else {
					mMouse.mState ^= MOUSE_SELECTED;
				}
			}
			else
				mMouse.mState ^= MOUSE_LEFT;	
		}
			break;
		case 1:{
			if (action){
				mMouse.mState |= MOUSE_RIGHT;
			}
			else {
				mMouse.mState ^= MOUSE_RIGHT;	
			}
			break;
		}
		
		case 2:{
			if (action){
				mMouse.mState |= MOUSE_MIDDLE;
			}
			else {
				mMouse.mState ^= MOUSE_MIDDLE;	
			}
			break;
		}
	}
	//cout << hex << (unsigned short int) mMouseState << endl;
}

/*
 * Deal with the mouse wheel movement, 2d or otherwise
 */
 
static float_t wsense = 10.0f; 
 
void Leeds::mouseWheelCallback(int xpos, int ypos) {
	pInfo->d.zoomCamera(ypos * wsense);
	mMouse.mWheel = ypos;
}


/*
 * Leeds Key Callback
 */
 
 
void Leeds::keyCallback(int key, int action){
	if (key == 84 && action == 1){
		toggleShowCameras();
	}
}
 

/*
 * Painting the GL thing
 */

void Leeds::draw(double_t dt) {

	//double ddt = sampleClock(1);
	
	glClearBufferfv(GL_COLOR, 0, &glm::vec4(0.9f, 0.9f, 0.9f, 1.0f)[0]);
	GLfloat depth = 1.0f;
	glClearBufferfv(GL_DEPTH, 0, &depth );
	
	mManager.update(); // Placed here as it has opengl texture calls
	if (qState.size() > 0)
		qState.back().draw();
	
	// Draw the list states
	for (state_iterator it = lState.begin(); it != lState.end(); ++it){
		it->draw();
	}
	
	TwDraw();
}

/*
 * Actions for states - Toggling the view Cameras
 */
 
void Leeds::toggleShowCameras() {
	ListState<StateDisplayCameras> s(lState,pInfo);
	if ( hasState(lState, s.mID) == lState.end())
		s();
	else
		s.remove();
}

/*
 * Toggle the scanning state
 */

void Leeds::toggleScanning() {
	StackState<StateScan> s(qState,pInfo);
	if (!s.remove())
		s();
}

/*
 * Toggle Texturing State
 */
 
void Leeds::toggleTexturing(){
	
	StackState<StateTexturing> s(qState,pInfo);
	if (!s.remove())
		s();

}

/*
 * Fire up the calibrate world
 */
 
 void Leeds::calibrateWorld() {
	StackState<StateCalibrateWorld> s(qState,pInfo);
	if (!s.remove())
		s();
 }
 
 /*
  * Fire up the calibrate Cameras
  */
 
 void Leeds::calibrateCameras() {
	StackState<StateCalibrateCamera> s(qState,pInfo);
	if (!s.remove())
		s();
 }
 
 /*
  * Toggle the detected points
  */
  
 void Leeds::toggleDetected(){
	if (qState.back().mID == "StateScan"){
		pInfo->sr = !pInfo->sr;
	}
 }
 
 /*
  * Save Meshes to file
  */
  
void Leeds::save() {
	mM.generate(mManager.getCams());
	mM.saveToFile("./data/test.pcd");
	mM.saveMeshToFile("./data/test.stl");
}

void Leeds::load(std::string filename) {
	 mM.loadFile(filename);
}

/*
 * Clear the Mesh
 */
 
void Leeds::clearMesh() {mM.clearMesh(); }

/*
 * Generate Mesh
 */

void Leeds::generateMesh() { mM.generate(pInfo->c.getCams()); }

/*
 * Toggle drawing the mesh filled in
 */
 
void Leeds::toggleDrawMesh() { 
ListState<StateDrawMesh> s(lState,pInfo);
	if ( hasState(lState, s.mID) == lState.end())
		s();
	else
		s.remove();
}
