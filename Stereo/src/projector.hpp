/**
* @brief Projector Window
* @file projector.hpp
* @date 04/07/2012
*
*/

#ifndef PROJECTOR_HPP
#define PROJECTOR_HPP

#include <glm/glm.hpp>

#include "drawer.hpp"
#include "config.hpp"
#include "app.hpp"

class Projector : public AppProvider {
public:
	Projector(GlobalConfig &mConfig);

	void draw(double_t dt);
	void init();
	void resize(int w, int h);
	void keyCallback(int key, int action);
	void mousePositionCallback(int x, int y) {};
	void mouseButtonCallback(int button, int action){};
	void mouseWheelCallback(int xpos, int ypos) {};

	void setWindowed(int w, int h);
	void setFullscreen();

protected:
	
	Shader mShaderQuad;
	
	GlobalConfig &mConfig;
	
	VBOData mQuad;
	bool mF;
	int mW,mH;
	float_t mPx,mPy;
	
};


#endif
