/**
* @brief Projector Window
* @file projector.cpp
* @date 04/07/2012
*
*/

#include "projector.hpp"

using namespace std;
using namespace boost;
using namespace boost::assign; 

Projector::Projector(GlobalConfig &config) : mConfig(config) {
	mF = false;
}

static float_t quadsize = 5.0;

void Projector::draw(double_t dt){
	glClearBufferfv(GL_COLOR, 0, &glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)[0]);
	GLfloat depth = 1.0f;
	glClearBufferfv(GL_DEPTH, 0, &depth );
	
	if (mW > 0 && mH > 0 && mWindow) { 
		
		mPy = mPx > mW ? mPy + quadsize : mPy;
		mPy = mPy > mH ? 0.0 : mPy;
		mPx = mPx > mW ? 0.0 : mPx + (mConfig.scanInterval * quadsize * dt);
				
		glm::mat4 Projection = glm::ortho(0.0f,(float_t)mW,(float_t)mH, 0.0f);
		glm::mat4 Model = glm::translate(glm::mat4(1.0f), glm::vec3(mPx,mPy, 0.0));
		glm::mat4 MVP = Projection * Model;
		
		mQuad.bind();
		mShaderQuad.begin();
			
		GLint LocationMVP = glGetUniformLocation(mShaderQuad.getProgram(), "mMVPMatrix");

		glUniformMatrix4fv(	LocationMVP, 1, GL_FALSE, glm::value_ptr(MVP)); 
		glDrawElements(GL_TRIANGLES, mQuad.mNumIndices, GL_UNSIGNED_INT, 0);
		
		mShaderQuad.end();
		mQuad.unbind();
		
	}
	
}


void Projector::init(){
	mQuad.mIndices += 0,3,1,3,2,1;
	mQuad.mVertices += 0.0f,0.0f,0.0f,
		quadsize, 0.0f,0.0f, 
		quadsize, quadsize,0.0f,
		0.0f, quadsize,0.0f;
	
	mQuad.mColours += 1.0,1.0,1.0,1.0,
			1.0,1.0,1.0,1.0,
			1.0,1.0,1.0,1.0,
			1.0,1.0,1.0,1.0;
		
	mQuad.compile(VBO_VERT | VBO_IDCE | VBO_COLR );
	
	mShaderQuad.load("./data/quad.vert", "./data/quad.frag");
	
	 mPx = mPy = 0;

}

void Projector::resize(int w, int h){
	mW = w;
	mH = h;
	glViewport(0,0,w,h);
}

void Projector::keyCallback(int key, int action){
	
	if (key == 70 && action == 0){
		if (!mF)
			setFullscreen();
		else
			setWindowed(0,0);
		mF = !mF;
	}
}


/*
 * Go to windowed mode 
 * As of master branch, this isnt supported in GLFW 3 but is in a seperate branch
 */

void Projector::setWindowed(int w, int h){
	glfwCloseWindow(mWindow);
	mWindow = glfwOpenWindow(0,0, GLFW_WINDOWED, mTitle.c_str(), NULL);
	init();
}

/*
 * Go to windowed mode 
 * As of master branch, this isnt supported in GLFW 3 but is in a seperate branch
 */


void Projector::setFullscreen(){
	cout << "Leeds - Projector going fullscreen" << endl;
	glfwCloseWindow(mWindow);
	GLFWvidmode mode;
	glfwGetDesktopMode( &mode );
	mW = mode.width;
	mH = mode.height;

	mWindow = glfwOpenWindow(mode.width,mode.height, GLFW_FULLSCREEN, mTitle.c_str(), NULL);
	init();
}


