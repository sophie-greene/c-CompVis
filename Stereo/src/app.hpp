/**
* @brief GLFW Window based solution
* @file app.hpp
* @date 03/07/2012
*
*/

#ifndef GLAPP_HPP
#define GLAPP_HPP


#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/foreach.hpp>
#include <gtkmm.h>

// Before glfw3 as this includes glew
#include "config.hpp"

// Using the multimonitor branch which works with X11
#include <GL/glfw3.h>
#include <anttweakbar/AntTweakBar.h>

/*
 * Struct to define an area that is passed to renderers
 * \todo if we get fullscreen across different windows we need this
 */
 
struct GLArea {
	size_t mX,mY,mW,mH;
};


/*
 * Base class for Applications that need to be drawn to a window
 */
 
class AppProvider {
public:
	virtual void show(std::string title="GL App") {
		mTitle = title; 
		mWindow = glfwOpenWindow(800,600, GLFW_WINDOWED, title.c_str(), NULL);
		resize(800,600);
	};
	virtual void init(){};
	virtual void draw(double_t dt) = 0;
	virtual void resize(int w, int h) = 0;
	virtual void keyCallback(int key, int action){};
	virtual void mousePositionCallback(int x, int y){};
	virtual void mouseButtonCallback(int button, int action){};
	virtual void mouseWheelCallback(int xpos, int ypos){};
	
	GLFWwindow mWindow;
	size_t mH, mW;
	std::string mTitle;
	
};

typedef boost::shared_ptr<AppProvider> AppPtr;
typedef std::vector<boost::shared_ptr<AppProvider> >::iterator AppItr;


/*
 * The main Application. Creates windows with GLFW3 under X11
 */

class GLApp{
public:
	GLApp();
	
	static void mainLoop();
	static void init();
	static void reshape(GLFWwindow window, int w, int h);

	// OS Dependent
	static void loadFile();

	static void keyCallback(GLFWwindow window, int key, int action);
	static void mouseButtonCallback(GLFWwindow window, int button, int action);
	static void mousePositionCallback(GLFWwindow window, int x, int y);
	static void mouseWheelCallback(GLFWwindow window, int xpos, int ypos);
	static void monitorCallback( GLFWmonitor m, int p);
	
protected:

	static AppPtr mLeeds;
	static AppPtr mProjector;

	static std::vector<AppPtr> vWindows;
	
	static GLboolean mRunning;
		
};


#endif

