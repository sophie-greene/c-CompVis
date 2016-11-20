/**
* @brief Drawing Class
* @file drawer.hpp
* @date 26/04/2012
*
*/


#ifndef _DRAWER_HPP_
#define _DRAWER_HPP_

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "camera_manager.hpp"
#include "utils.hpp"
#include "config.hpp"


/*
 * Basic Shader class - loads and binds
 */

class Shader {
public:
	void load(std::string vert, std::string frag);
	GLuint getProgram() { return mProgram; };
	void begin() { glUseProgram(mProgram);};
	void end() {glUseProgram(0);};
	
	~Shader() { glDetachShader(mProgram, mVS); glDetachShader(mProgram, mFS);  } 
	
protected:
   
    GLuint mVS, mFS;
    GLuint mProgram;

};

/*
 * Basic FBO Class
 */
 
class FBO {
public:
	void setup(size_t w, size_t h);
	void bind() { glBindFramebuffer(GL_FRAMEBUFFER, mID); };
	void unbind() { glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0); } ;
	bool checkStatus();
	void printFramebufferInfo();
	void resize(size_t w, size_t h);
	
protected:
	GLuint mW,mH,mID,mDepth,mColour;

};

/*
 * Struct for primitive objects with their own co-ordinates
 * \todo maybe pass in shaders here as well?
 */
 
class Primitive {
public:
	virtual ~Primitive(); 
	
	void generate(std::string filename, glm::vec3 scale);
	void move(glm::vec3 p) {mPos += p; compute(); };
	void rotate(glm::vec3 r) {mRot += r; compute(); };
	glm::mat4 getMatrix() { return mTransMatrix * mRotMatrix * mScaleMatrix; };
	void compute();
	int loadAsset(std::string filename);
	
	void bind() { mVBO.bind(); };
	void unbind() {mVBO.unbind(); };
	GLuint getNumElements() { return mVBO.mNumElements; };

protected:
	void recursiveCreate (const struct aiScene *sc, const struct aiNode* nd);
	
	VBOData mVBO;
	
	const struct aiScene* pScene;
	
	glm::vec3 mPos;
	glm::vec3 mRot;
	glm::vec3 mScale;
	
	glm::mat4 mTransMatrix;
	glm::mat4 mRotMatrix;
	glm::mat4 mScaleMatrix;

};


/* 
 * Camera Class Should probably copy this from Cinder but hey
 */
 
class Camera {

public:

	Camera();
	void yaw(float_t a);
	void pitch(float_t a);
	void roll(float_t a);
	void setRatio(float_t r);
	
	virtual void move(glm::vec3 m);
	
	glm::vec3 getPos() {return mPos; };
	glm::vec3 getLook() {return mLook; };
	glm::vec3 getUp() {return mUp; };
	
	glm::mat4 getViewMatrix() { return mViewMatrix; };
	glm::mat4 getProjMatrix() { return mProjectionMatrix; };

protected:
	virtual void compute(float_t ax, float_t ay, float_t az);

	glm::mat4 mViewMatrix;
	glm::mat4 mProjectionMatrix;
	
	glm::vec3 mPos;
	glm::vec3 mLook;
	glm::vec3 mUp;
	
	float_t mR;
	
};

/* 
 * Orbit camera revolves around one point as it were
 */
 
class OrbitCamera : public Camera{
public:
	OrbitCamera();
	void setTarget(glm::vec3 l) { mLook = l; compute(0,0,0); };
	void zoom(float_t z);
	void shift(float_t du, float_t dv);
	
protected:
	void compute(float_t ax, float_t ay, float_t az);

};

/*
 * Drawer class - Performs all drawing functions using OpenCV and GLM for its functionality
 * Loaders for shaders and VBO code is custom
 * We assume a graphics context has already been given to this class.
 * 
 * \todo could have one drawer per state technically? Different cameras and views allowed?
 * 
 */

class Drawer {
public:
	Drawer() {};
	virtual ~Drawer();
	void setup(GlobalConfig &config);
	void resize(size_t w, size_t h);
	void drawCamerasFlat(std::vector<boost::shared_ptr<LeedsCam> > &cams);
	void drawMesh(VBOData &mesh, std::vector<boost::shared_ptr<LeedsCam> > &cams);
	void drawMeshPoints(VBOData &points, float r, float g, float b);
	void drawResultFlat(size_t cami);
	void drawCameraQuad();
	void drawCameraNormal(VBOData &line);
	void drawNormals(VBOData &lines);
	void drawReferenceQuad();
	void drawGripper();
	void rotateCamera(int dx, int dy, double dt);
	void zoomCamera(float_t z);
	void moveCamera(int dx, int dy, double dt);
	bool picked(size_t u, size_t v, size_t s, glm::vec3 col);
	void moveGripper(float_t du, float_t dv, float_t dt);

protected:

	void recursiveCreate (const struct aiScene *sc, const struct aiNode* nd);
	
	struct SharedObj {
		SharedObj(GlobalConfig &config) : mConfig(config) {};
		GlobalConfig &mConfig;
		size_t mW,mH;
		
		//Shaders
		Shader mShaderFlatCam;
		Shader mShaderMeshPoints;
		Shader mShaderQuad;
		Shader mShaderLighting;
		Shader mShaderNormals;
		Shader mShaderGripper;
		Shader mShaderPicker;
				
		glm::mat4 mModelMatrix;
		
		OrbitCamera mCam;
		
		struct aiLogStream mStream;
		
		FBO	mFBOPick;
		
		VBOData	mVCam;
		VBOData	mVQuad;
		Primitive mGripper;
	
		
	};
	
	
	boost::shared_ptr<SharedObj> mObj;
};

#endif
