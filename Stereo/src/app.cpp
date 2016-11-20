/**
* @brief GLFW Window based solution
* @file app.hpp
* @date 03/07/2012
*
*/

#include "app.hpp"

#include "leeds.hpp"
#include "projector.hpp"

using namespace std;

AppPtr GLApp::mLeeds;
AppPtr GLApp::mProjector;
	
std::vector<AppPtr> GLApp::vWindows;
GLboolean GLApp::mRunning;

/*
 * Create our Windows and contexts
 */

GLApp::GLApp() {
	
	if( !glfwInit(NULL) ){
		fprintf( stderr, "Failed to initialize GLFW\n" );
		exit( EXIT_FAILURE );
	}

}
/*
 * Main Loop - calls the drawing methods and events for our mini apps
 */

void GLApp::mainLoop() {

	mRunning = GL_TRUE;
	double_t t = glfwGetTime();

	while (mRunning){
		bool ui = true;
		
		BOOST_FOREACH ( AppPtr b, vWindows) {	
			glfwMakeContextCurrent( b->mWindow);
			b->draw(glfwGetTime() - t);
			glfwSwapBuffers();
		}
		t = glfwGetTime();
		
	
		glfwPollEvents();
		
		// Linux only - need equivalent for Mac/Windows
		gtk_main_iteration_do(false);
		
		BOOST_FOREACH ( AppPtr b, vWindows) {
			if (!glfwIsWindow(b->mWindow))
				mRunning = GL_FALSE;
		}
    }
    
	boost::shared_ptr<Leeds> l = boost::static_pointer_cast<Leeds>(mLeeds);
    l->quit();
}

/*
 * Key Callback - Check the window against the apps and call the app
 */
 
void GLApp::keyCallback(GLFWwindow window, int key, int action){
	
	cout << key << "," << action << endl;
	
	if (key == 76 && action ==1){
		loadFile();
	}
	
	BOOST_FOREACH ( AppPtr b, vWindows) {
		if (b->mWindow == window){
			b->keyCallback(key,action);
			
			if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
				glfwCloseWindow(window);
		}
	}
}

/*
 * Mouse Callback for Button
 */


void GLApp::mouseButtonCallback(GLFWwindow window, int button, int action) {
	if (!TwEventMouseButtonGLFW(button,action)){
		BOOST_FOREACH ( AppPtr b, vWindows) {
			if (b->mWindow == window){
				b->mouseButtonCallback(button,action);
			}
		}
	}
}

/*
 * Mouse Callback for Position
 */

void GLApp::mousePositionCallback(GLFWwindow window, int x, int y) {
	if( !TwEventMousePosGLFW(x, y) ){  
	 	 
		BOOST_FOREACH ( AppPtr b, vWindows) {
			if (b->mWindow == window){
				b->mousePositionCallback(x,y);
			}
		}
   
    //printf("%08x at %0.3f: Mouse position: %i %i\n", 0, glfwGetTime(), x, y);
	}
}

/*
 * Mouse Wheel Callback
 */
 
void GLApp::mouseWheelCallback(GLFWwindow window, int xpos, int ypos){
  BOOST_FOREACH ( AppPtr b, vWindows) {
		if (b->mWindow == window){
			b->mouseWheelCallback(xpos,ypos);
		}
	}
}

/*
 * File Dialog running with GTK
 */
 
void GLApp::loadFile() {
	Gtk::FileChooserDialog dialog("Please choose a Mesh File", Gtk::FILE_CHOOSER_ACTION_OPEN);

	//Add response buttons the the dialog:
	dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	dialog.add_button("Select", Gtk::RESPONSE_OK);
	
	/*Glib::RefPtr<Gtk::FileFilter> filter_text = Gtk::FileFilter::create();
	filter_text->set_name("PCD Files");
	filter_text->add_mime_type("text/plain");
	dialog.add_filter(filter_text);*/

	int result = dialog.run();

	//Handle the response:
	switch(result) {
		case(Gtk::RESPONSE_OK): {
			boost::shared_ptr<Leeds> l = boost::static_pointer_cast<Leeds>(mLeeds);
			l->load(dialog.get_filename());
			break;
		}
		case(Gtk::RESPONSE_CANCEL): {
			break;
		}
		default:{
			break;
		}
	}
	
}
/*
 * Monitor Callback
 */
 
void GLApp::monitorCallback( GLFWmonitor m, int p){
	cout << "Monitor " << p << endl;
}


/*
 * Fire the reshape event for a window
 */
 
void GLApp::reshape(GLFWwindow window, int w, int h ) {
	BOOST_FOREACH ( AppPtr b, vWindows) {
		if (b->mWindow == window){
			b->resize(w,h);
		}
	}
}

/*
 * Init the applications now we have a GL Context
 */
 
void GLApp::init() {
	

	// Init the Leeds Application
	
	mLeeds = AppPtr(new Leeds);
	
	glfwOpenWindowHint(GLFW_OPENGL_VERSION_MAJOR, 4);
	glfwOpenWindowHint(GLFW_OPENGL_VERSION_MINOR, 0);
	
	///\todo fully switch to core profile - there is something causing an error in core
	glfwOpenWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
	glfwOpenWindowHint(GLFW_FSAA_SAMPLES, 4);
	
	mLeeds->show("Leeds");
	
	cout << "Leeds OpenGL Version: " << glGetString(GL_VERSION) << endl;
	
	glfwSetKeyCallback(keyCallback);
	glfwSetMousePosCallback(mousePositionCallback);
	glfwSetMouseButtonCallback(mouseButtonCallback);
	glfwSetScrollCallback(mouseWheelCallback);
	glfwSetWindowSizeCallback(reshape);
	glfwSetMonitorDeviceCallback(monitorCallback);

    glfwSwapInterval(1);

	
	if( !mLeeds->mWindow ) {
		fprintf( stderr, "Failed to open GLFW window\n" );
		glfwTerminate();
		exit( EXIT_FAILURE );
	}
	
	// Call only after one window / context has been created!
	
	glewExperimental = true;
	GLenum err=glewInit();

	if(err!=GLEW_OK) {
		//Problem: glewInit failed, something is seriously wrong.
		cout << "Leeds - GLEWInit failed, aborting." << endl;
		glfwTerminate();
		exit( EXIT_FAILURE );
		
	}
	TwInit(TW_OPENGL, NULL);
	
	// Now we can commit
	
	mLeeds->init();
	vWindows.push_back(mLeeds);
	boost::shared_ptr<Leeds> l = boost::static_pointer_cast<Leeds>(mLeeds);
	
	
	
	// Create the projector
	
	mProjector = AppPtr(new Projector(l->mConfig) );
	vWindows.push_back(mProjector);
	glfwOpenWindowHint(GLFW_FSAA_SAMPLES, 0);
	mProjector->show("Projection");
	mProjector->init();
	
}



int main(int argc, char *argv[]) {
   
	// Linux only - need equivalent for Mac / Windows
	Gtk::Main kit(argc, argv);
	
	GLApp app;
  
	app.init();

	app.mainLoop();

	glfwTerminate();
	
	exit( EXIT_SUCCESS );
}
